/**
 * screenshot.h — Save display framebuffer as PNG to SD card.
 *
 * Reads the DPI panel framebuffer (RGB565), converts to RGB888,
 * and writes a valid PNG file using ROM zlib (miniz) deflate compression.
 * Processes one row at a time to keep RAM usage low (~70KB working set).
 *
 * Usage: screenshot_to_sd(panel, width, height)
 *   - Call from the `screenshot` / `ss` serial command
 *   - Requires LVGL API lock held (for display stability)
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <sys/stat.h>
#include "esp_log.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
extern "C" {
#include "miniz.h"
}

static const char *SS_TAG = "SCREENSHOT";
static char ss_last_filename[80] = "";

// ─── CRC32 (PNG standard polynomial) ────────────────────────────────────────

static uint32_t ss_crc32_table[256];
static bool ss_crc32_ready = false;

static void ss_crc32_build_table(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c >> 1) ^ (c & 1 ? 0xEDB88320 : 0);
        ss_crc32_table[i] = c;
    }
    ss_crc32_ready = true;
}

static uint32_t ss_crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    if (!ss_crc32_ready) ss_crc32_build_table();
    for (size_t i = 0; i < len; i++)
        crc = ss_crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

// ─── PNG helpers ─────────────────────────────────────────────────────────────

static void ss_write_be32(uint8_t *p, uint32_t v) {
    p[0] = (v >> 24); p[1] = (v >> 16); p[2] = (v >> 8); p[3] = v;
}

// Write a PNG chunk: length(4) + type(4) + data(length) + crc32(4)
static bool ss_write_chunk(FILE *f, const char *type, const uint8_t *data, uint32_t len) {
    uint8_t hdr[8];
    ss_write_be32(hdr, len);
    memcpy(hdr + 4, type, 4);
    if (fwrite(hdr, 1, 8, f) != 8) return false;

    uint32_t crc = 0xFFFFFFFF;
    crc = ss_crc32_update(crc, (const uint8_t *)type, 4);
    if (len > 0) {
        if (fwrite(data, 1, len, f) != len) return false;
        crc = ss_crc32_update(crc, data, len);
    }
    crc ^= 0xFFFFFFFF;

    uint8_t crc_buf[4];
    ss_write_be32(crc_buf, crc);
    return fwrite(crc_buf, 1, 4, f) == 4;
}

// ─── RGB565 → RGB888 row conversion ─────────────────────────────────────────

static void ss_rgb565_to_rgb888_row(const uint16_t *src, uint8_t *dst, int width) {
    for (int x = 0; x < width; x++) {
        uint16_t px = src[x];
        uint8_t r = (px >> 11) & 0x1F;
        uint8_t g = (px >> 5) & 0x3F;
        uint8_t b = px & 0x1F;
        dst[x * 3 + 0] = (r << 3) | (r >> 2);
        dst[x * 3 + 1] = (g << 2) | (g >> 4);
        dst[x * 3 + 2] = (b << 3) | (b >> 2);
    }
}

// ─── tdefl output callback state (static — screenshot is single-threaded) ───

static FILE     *ss_tdefl_file = NULL;
static uint32_t *ss_tdefl_crc  = NULL;
static size_t   *ss_tdefl_len  = NULL;
static bool      ss_tdefl_ok   = true;

static mz_bool ss_tdefl_put_buf(const void *buf, int len, void *user) {
    (void)user;
    if (len > 0 && ss_tdefl_ok) {
        if (fwrite(buf, 1, len, ss_tdefl_file) != (size_t)len) {
            ss_tdefl_ok = false;
            return MZ_FALSE;
        }
        *ss_tdefl_crc = ss_crc32_update(*ss_tdefl_crc, (const uint8_t *)buf, len);
        *ss_tdefl_len += len;
    }
    return MZ_TRUE;
}

// ─── Main screenshot function ────────────────────────────────────────────────

/**
 * Save the current display framebuffer as a PNG file on the SD card.
 *
 * Uses ROM zlib (miniz) for deflate compression — typically 3-5x smaller
 * than uncompressed stored blocks. Processes one row at a time.
 *
 * @param panel   ESP LCD DPI panel handle (for get_frame_buffer)
 * @param width   Display width in pixels
 * @param height  Display height in pixels
 * @return true on success
 */
static inline bool screenshot_to_sd(esp_lcd_panel_handle_t panel,
                                     uint32_t width, uint32_t height) {
    if (!panel) {
        ESP_LOGE(SS_TAG, "No panel handle");
        return false;
    }

    // Get the DPI framebuffer pointer (first buffer)
    void *fb = NULL;
    esp_err_t err = esp_lcd_dpi_panel_get_frame_buffer(panel, 1, &fb);
    if (err != ESP_OK || !fb) {
        ESP_LOGE(SS_TAG, "Failed to get framebuffer: 0x%x", err);
        return false;
    }

    // ── Snapshot framebuffer to PSRAM ──
    // The DPI panel is live — LVGL repaints during the ~1.3s PNG encode.
    // Copy the entire framebuffer quickly under the LVGL lock so we get
    // a consistent frame, then release the lock and encode from the copy.
    size_t fb_size = (size_t)width * height * sizeof(uint16_t);
    uint16_t *fb_copy = (uint16_t *)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!fb_copy) {
        ESP_LOGE(SS_TAG, "Failed to alloc framebuffer copy (%zu bytes)", fb_size);
        return false;
    }

    extern _lock_t lvgl_api_lock;
    _lock_acquire(&lvgl_api_lock);
    memcpy(fb_copy, fb, fb_size);
    _lock_release(&lvgl_api_lock);

    // Allocate row buffer: filter byte + RGB888 pixels
    size_t row_bytes = 1 + width * 3;
    uint8_t *row_buf = (uint8_t *)heap_caps_malloc(row_bytes, MALLOC_CAP_SPIRAM);
    if (!row_buf) {
        ESP_LOGE(SS_TAG, "Failed to alloc row buffer (%zu bytes)", row_bytes);
        heap_caps_free(fb_copy);
        return false;
    }

    // Build filename with UTC timestamp
    mkdir("/sdcard/screenshots", 0755);

    char filename[80];
    time_t now;
    time(&now);
    struct tm t;
    gmtime_r(&now, &t);
    if (now > 1700000000) {
        strftime(filename, sizeof(filename),
                 "/sdcard/screenshots/screenshot_%Y%m%d_%H%M%S.png", &t);
    } else {
        snprintf(filename, sizeof(filename),
                 "/sdcard/screenshots/screenshot_%lu.png",
                 (unsigned long)(esp_timer_get_time() / 1000000));
    }

    FILE *f = fopen(filename, "wb");
    if (!f) {
        ESP_LOGE(SS_TAG, "Failed to open %s", filename);
        heap_caps_free(row_buf);
        heap_caps_free(fb_copy);
        return false;
    }
    strncpy(ss_last_filename, filename, sizeof(ss_last_filename) - 1);

    int64_t t0 = esp_timer_get_time();

    // ── PNG signature ──
    static const uint8_t png_sig[8] = {0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A};
    fwrite(png_sig, 1, 8, f);

    // ── IHDR chunk ──
    uint8_t ihdr[13];
    ss_write_be32(ihdr + 0, width);
    ss_write_be32(ihdr + 4, height);
    ihdr[8]  = 8;   // bit depth
    ihdr[9]  = 2;   // color type: RGB
    ihdr[10] = 0;   // compression: deflate
    ihdr[11] = 0;   // filter: adaptive
    ihdr[12] = 0;   // interlace: none
    ss_write_chunk(f, "IHDR", ihdr, 13);

    // ── IDAT chunk — compressed with ROM miniz tdefl ──
    //
    // The ESP32-P4 ROM miniz has MINIZ_NO_ZLIB_APIS, so we use the low-level
    // tdefl compressor directly with a callback that writes to file.
    // TDEFL_WRITE_ZLIB_HEADER adds the zlib wrapper PNG requires.
    //
    // We don't know the compressed size upfront, so:
    //   1. Write placeholder IDAT length + "IDAT"
    //   2. Stream deflated data via callback, tracking CRC and byte count
    //   3. Seek back and patch the length
    //
    long idat_length_pos = ftell(f);
    uint8_t idat_hdr[8] = {0,0,0,0, 'I','D','A','T'};
    fwrite(idat_hdr, 1, 8, f);

    uint32_t chunk_crc = 0xFFFFFFFF;
    chunk_crc = ss_crc32_update(chunk_crc, (const uint8_t *)"IDAT", 4);
    size_t idat_data_len = 0;

    // Allocate tdefl_compressor in PSRAM (~200KB struct)
    tdefl_compressor *comp = (tdefl_compressor *)heap_caps_malloc(
        sizeof(tdefl_compressor), MALLOC_CAP_SPIRAM);
    if (!comp) {
        ESP_LOGE(SS_TAG, "Failed to alloc tdefl_compressor (%zu bytes)", sizeof(tdefl_compressor));
        fclose(f);
        heap_caps_free(row_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    // Compression flags: level 6 = 128 probes (bits 0–12), no greedy parsing.
    // TDEFL_WRITE_ZLIB_HEADER adds the zlib wrapper PNG IDAT requires.
    int comp_flags = 128 | TDEFL_WRITE_ZLIB_HEADER;
    ss_tdefl_file = f;
    ss_tdefl_crc = &chunk_crc;
    ss_tdefl_len = &idat_data_len;
    ss_tdefl_ok = true;

    tdefl_status tst = tdefl_init(comp, ss_tdefl_put_buf, NULL, comp_flags);
    if (tst != TDEFL_STATUS_OKAY) {
        ESP_LOGE(SS_TAG, "tdefl_init failed: %d", tst);
        heap_caps_free(comp);
        fclose(f);
        heap_caps_free(row_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    const uint16_t *fb16 = fb_copy;
    bool ok = true;

    for (uint32_t y = 0; y < height && ok; y++) {
        // Build row: filter_byte(0x00 = None) + RGB888 pixels
        row_buf[0] = 0;
        ss_rgb565_to_rgb888_row(&fb16[y * width], row_buf + 1, width);

        // Feed row to deflate — TDEFL_FINISH on last row
        tdefl_flush flush = (y == height - 1) ? TDEFL_FINISH : TDEFL_NO_FLUSH;
        tst = tdefl_compress_buffer(comp, row_buf, row_bytes, flush);
        if (tst < TDEFL_STATUS_OKAY) {
            ESP_LOGE(SS_TAG, "tdefl_compress error: %d at row %lu", tst, (unsigned long)y);
            ok = false;
        }
        if (!ss_tdefl_ok) ok = false;
    }

    heap_caps_free(comp);
    heap_caps_free(fb_copy);  // framebuffer snapshot no longer needed

    if (ok) {
        // Write IDAT CRC
        chunk_crc ^= 0xFFFFFFFF;
        uint8_t crc_buf[4];
        ss_write_be32(crc_buf, chunk_crc);
        fwrite(crc_buf, 1, 4, f);

        // Patch IDAT length
        long end_pos = ftell(f);
        fseek(f, idat_length_pos, SEEK_SET);
        uint8_t len_buf[4];
        ss_write_be32(len_buf, (uint32_t)idat_data_len);
        fwrite(len_buf, 1, 4, f);
        fseek(f, end_pos, SEEK_SET);

        // ── IEND chunk ──
        ss_write_chunk(f, "IEND", NULL, 0);
    }

    fclose(f);
    heap_caps_free(row_buf);

    if (!ok) {
        remove(filename);
        return false;
    }

    int64_t dt = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(SS_TAG, "Saved %s (%lux%lu, %zu bytes compressed, %lld ms)",
             filename, (unsigned long)width, (unsigned long)height,
             idat_data_len, (long long)dt);

    printf("Screenshot saved: %s (%zu bytes, %lld ms)\n",
           filename, idat_data_len + 57, (long long)dt);
    return true;
}

// ─── Base64 encoding for serial transport ────────────────────────────────────

static const char ss_b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t ss_base64_encode(const uint8_t *in, size_t in_len, char *out) {
    size_t pos = 0;
    for (size_t i = 0; i < in_len; ) {
        uint32_t a = i < in_len ? in[i++] : 0;
        uint32_t b = i < in_len ? in[i++] : 0;
        uint32_t c = i < in_len ? in[i++] : 0;
        uint32_t triple = (a << 16) | (b << 8) | c;
        out[pos++] = ss_b64_table[(triple >> 18) & 0x3F];
        out[pos++] = ss_b64_table[(triple >> 12) & 0x3F];
        out[pos++] = (i > in_len + 1) ? '=' : ss_b64_table[(triple >> 6) & 0x3F];
        out[pos++] = (i > in_len)     ? '=' : ss_b64_table[triple & 0x3F];
    }
    out[pos] = '\0';
    return pos;
}

/**
 * Read a PNG file from SD and send it over serial as base64.
 *
 * Output format (webapp-parseable):
 *   SCREENSHOT_BEGIN <filename> <size_bytes>
 *   <base64 lines, 76 chars each>
 *   SCREENSHOT_END
 *
 * @param path  Path to the PNG file on SD card
 * @return true on success
 */
static inline bool screenshot_send_serial(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(SS_TAG, "Failed to open %s for serial send", path);
        return false;
    }

    // Get file size
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    // Extract just the filename from the path
    const char *fname = strrchr(path, '/');
    fname = fname ? fname + 1 : path;

    printf("SCREENSHOT_BEGIN %s %ld\n", fname, file_size);

    // Read 57 bytes at a time → 76 base64 chars per line (standard)
    uint8_t raw[57];
    char b64[80];
    size_t total_sent = 0;

    while (total_sent < (size_t)file_size) {
        size_t to_read = sizeof(raw);
        if (total_sent + to_read > (size_t)file_size)
            to_read = file_size - total_sent;

        size_t got = fread(raw, 1, to_read, f);
        if (got == 0) break;

        ss_base64_encode(raw, got, b64);
        printf("%s\n", b64);
        total_sent += got;
    }

    printf("SCREENSHOT_END\n");
    fclose(f);

    ESP_LOGI(SS_TAG, "Sent %zu bytes as base64 over serial", total_sent);
    return true;
}
