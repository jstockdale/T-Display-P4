/*
 * music_player.cpp — SD card music player for T-Display-P4
 *
 * MP3 decoding via minimp3 (public domain, single header).
 * WAV support for 16-bit PCM.
 * ID3v2 tag parsing for title/artist metadata.
 */

#define MINIMP3_IMPLEMENTATION
#include "minimp3.h"
#include "music_player.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <ctype.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

// ES8311 I2S write — provided by cpp_bus_driver
#include "cpp_bus_driver_library.h"
extern Cpp_Bus_Driver::Es8311 *ES8311;

static const char *TAG = "MUSIC";

// ─── State ──────────────────────────────────────────────────────────────────

static music_track_t *s_tracks = nullptr;  // PSRAM array
static int             s_track_count = 0;
static int             s_current_idx = 0;

static volatile music_state_t s_state = MUSIC_STATE_STOPPED;
static volatile bool   s_stop_requested = false;
static volatile bool   s_pause_requested = false;
static volatile bool   s_track_changed = false;
static volatile double s_seek_target = -1.0;

static double          s_position_s = 0.0;
static double          s_duration_s = 0.0;

// minimp3 decoder — allocated on PSRAM heap (>10KB struct)
static mp3dec_t *s_mp3d = nullptr;

// Persistent playback buffers — allocated once in init, never freed.
// Avoids per-track alloc/free cycles that fragment heap.
#define MP3_INBUF_SIZE (16 * 1024)
#define WAV_READ_SIZE  (8 * 1024)

// ─── Async Read-Ahead Buffer ────────────────────────────────────────────────
//
// Large PSRAM buffer between SD card and decoder. A dedicated FreeRTOS task
// handles all SD reads, so the decode loop only ever touches PSRAM (fast,
// bounded-time). This eliminates audio skips caused by SD SPI latency.
//
// Architecture:
//   Fill task (low priority):  fread → PSRAM, wakes on notification
//   Decode loop (audio task):  PSRAM memcpy only, notifies fill task when room
//   Mutex protects shared state (held only for fast PSRAM ops, never during fread)

#define READAHEAD_SIZE  (512 * 1024)   // 512KB PSRAM ring buffer
#define READAHEAD_CHUNK (64 * 1024)    // SD read size per fill iteration

static uint8_t  *s_readahead     = nullptr;   // PSRAM buffer
static size_t    s_readahead_len = 0;          // valid bytes in buffer
static size_t    s_readahead_pos = 0;          // read cursor within buffer
static long      s_readahead_base = 0;         // file offset of s_readahead[0]
static FILE     *s_readahead_fp  = nullptr;    // file being read (fill task only during fread)
static bool      s_readahead_eof = false;       // hit end of file
static size_t    s_readahead_total_sd = 0;     // diagnostic: total bytes from SD

static SemaphoreHandle_t s_ra_mutex    = nullptr;   // protects shared buffer state
static TaskHandle_t      s_ra_task_hdl = nullptr;   // fill task handle
static volatile bool     s_ra_filling  = false;     // true while fread in progress
static volatile bool     s_ra_stop     = false;     // signals fill task to pause for close/seek
static volatile uint32_t s_ra_gen      = 0;         // generation counter — incremented on open/seek

// ─── Fill Task ──────────────────────────────────────────────────────────────

static void readahead_fill_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Readahead fill task started");
    for (;;) {
        // Sleep until woken by readahead_read/open/seek, or poll every 50ms
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

        if (s_ra_stop) continue;  // paused for close/seek
        if (!s_readahead || !s_readahead_fp || s_readahead_eof) continue;

        // Phase 1: compact buffer under mutex (PSRAM memmove, <1ms)
        xSemaphoreTake(s_ra_mutex, portMAX_DELAY);
        if (s_readahead_pos > 0) {
            size_t remaining = s_readahead_len - s_readahead_pos;
            if (remaining > 0)
                memmove(s_readahead, s_readahead + s_readahead_pos, remaining);
            s_readahead_base += s_readahead_pos;
            s_readahead_len = remaining;
            s_readahead_pos = 0;
        }
        size_t cur_len = s_readahead_len;
        FILE *fp = s_readahead_fp;
        uint32_t gen = s_ra_gen;  // snapshot generation before fread
        xSemaphoreGive(s_ra_mutex);

        // Phase 2: read from SD — NO mutex held (this is the slow ~30-65ms part).
        if (cur_len >= READAHEAD_SIZE || !fp) continue;

        size_t space = READAHEAD_SIZE - cur_len;
        size_t to_read = space < READAHEAD_CHUNK ? space : READAHEAD_CHUNK;

        s_ra_filling = true;
        size_t got = fread(s_readahead + cur_len, 1, to_read, fp);
        s_ra_filling = false;

        // Phase 3: update length under mutex — but only if generation matches
        xSemaphoreTake(s_ra_mutex, portMAX_DELAY);
        if (s_ra_gen == gen) {
            s_readahead_len = cur_len + got;
            s_readahead_total_sd += got;
            if (got < to_read) {
                s_readahead_eof = true;
                ESP_LOGD(TAG, "RA fill: EOF (total_sd=%zu)", s_readahead_total_sd);
            }
        } else {
            ESP_LOGD(TAG, "RA fill: stale gen=%lu now=%lu — discarded %zu bytes", (unsigned long)gen, (unsigned long)s_ra_gen, got);
        }
        xSemaphoreGive(s_ra_mutex);
    }
}

// ─── Wait for fill task to finish any in-progress fread ─────────────────────

static void readahead_wait_idle(void) {
    s_ra_stop = true;
    while (s_ra_filling) {
        vTaskDelay(1);
    }
    // Now fill task is either sleeping or spinning on s_ra_stop — safe to touch state
}

static void readahead_resume(void) {
    s_ra_stop = false;
    if (s_ra_task_hdl) xTaskNotifyGive(s_ra_task_hdl);
}

// ─── Open / Close ──────────────────────────────────────────────────────────

static void readahead_open(FILE *f) {
    readahead_wait_idle();

    xSemaphoreTake(s_ra_mutex, portMAX_DELAY);
    s_readahead_fp   = f;
    s_readahead_base = ftell(f);
    s_readahead_len  = 0;
    s_readahead_pos  = 0;
    s_readahead_eof  = false;
    s_readahead_total_sd = 0;
    s_ra_gen++;
    ESP_LOGI(TAG, "RA open: base=%ld gen=%lu", s_readahead_base, (unsigned long)s_ra_gen);
    xSemaphoreGive(s_ra_mutex);

    readahead_resume();
}

static void readahead_close(void) {
    readahead_wait_idle();

    xSemaphoreTake(s_ra_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "RA close: total_sd=%zu gen=%lu", s_readahead_total_sd, (unsigned long)(s_ra_gen + 1));
    if (s_readahead_fp) {
        fclose(s_readahead_fp);
        s_readahead_fp = nullptr;
    }
    s_readahead_len  = 0;
    s_readahead_pos  = 0;
    s_readahead_base = 0;
    s_readahead_eof  = false;
    s_readahead_total_sd = 0;
    s_ra_gen++;
    xSemaphoreGive(s_ra_mutex);

    s_ra_stop = false;  // allow task to sleep normally
}

// ─── Read (decode loop) — waits for data if buffer empty, never blocks on SD ──

static size_t readahead_read(void *dst, size_t bytes) {
    // If buffer is empty and not EOF, wait for fill task to deliver data.
    for (int retries = 0; retries < 100; retries++) {  // 100 ticks × ~10ms = ~1s max
        xSemaphoreTake(s_ra_mutex, portMAX_DELAY);
        size_t avail = s_readahead_len - s_readahead_pos;
        bool eof = s_readahead_eof;
        xSemaphoreGive(s_ra_mutex);

        if (avail > 0 || eof) break;

        // Buffer empty, not EOF — wake fill task and yield
        if (s_ra_task_hdl) xTaskNotifyGive(s_ra_task_hdl);
        vTaskDelay(1);
    }

    xSemaphoreTake(s_ra_mutex, portMAX_DELAY);

    size_t avail = s_readahead_len - s_readahead_pos;
    size_t to_copy = bytes < avail ? bytes : avail;
    if (to_copy > 0) {
        memcpy(dst, s_readahead + s_readahead_pos, to_copy);
        s_readahead_pos += to_copy;
    }
    avail -= to_copy;
    bool eof = s_readahead_eof;

    xSemaphoreGive(s_ra_mutex);

    // Wake fill task whenever there's room for a chunk
    if (s_ra_task_hdl && !eof && avail < (READAHEAD_SIZE - READAHEAD_CHUNK)) {
        xTaskNotifyGive(s_ra_task_hdl);
    }

    return to_copy;
}

// ─── Seek ──────────────────────────────────────────────────────────────────

static void readahead_seek(long offset, int whence) {
    if (!s_readahead_fp) return;

    readahead_wait_idle();

    xSemaphoreTake(s_ra_mutex, portMAX_DELAY);

    // Resolve absolute target
    long target;
    if (whence == SEEK_SET) {
        target = offset;
    } else if (whence == SEEK_CUR) {
        target = s_readahead_base + (long)s_readahead_pos + offset;
    } else {
        // SEEK_END: can't optimize
        s_readahead_len = 0;
        s_readahead_pos = 0;
        s_readahead_eof = false;
        s_readahead_total_sd = 0;
        s_ra_gen++;
        fseek(s_readahead_fp, offset, whence);
        s_readahead_base = ftell(s_readahead_fp);
        xSemaphoreGive(s_ra_mutex);
        readahead_resume();
        return;
    }

    // Check if target is within the currently buffered range
    long buf_start = s_readahead_base;
    long buf_end   = s_readahead_base + (long)s_readahead_len;

    if (target >= buf_start && target <= buf_end) {
        // Hit — just adjust position pointer (no SD access, no gen bump)
        s_readahead_pos = (size_t)(target - buf_start);
        xSemaphoreGive(s_ra_mutex);
        readahead_resume();
        return;
    }

    // Miss — invalidate buffer and seek the file
    s_readahead_len = 0;
    s_readahead_pos = 0;
    s_readahead_eof = false;
    s_readahead_total_sd = 0;
    s_ra_gen++;
    fseek(s_readahead_fp, target, SEEK_SET);
    s_readahead_base = target;

    xSemaphoreGive(s_ra_mutex);
    readahead_resume();
}

// Current logical file position
static long readahead_tell(void) {
    return s_readahead_base + (long)s_readahead_pos;
}

// Bytes available without SD access
static size_t readahead_available(void) {
    return s_readahead_len - s_readahead_pos;
}

// Wake fill task and wait until at least some data is buffered (or EOF).
// Used after open/seek to ensure the decode loop doesn't see an empty buffer
// and misinterpret it as end-of-file.
static void readahead_prefetch(void) {
    if (!s_ra_task_hdl) return;
    xTaskNotifyGive(s_ra_task_hdl);
    for (int i = 0; i < 100; i++) {  // 100 ticks × ~10ms = ~1s max wait
        if (readahead_available() > 0 || s_readahead_eof) return;
        vTaskDelay(1);
    }
    ESP_LOGW(TAG, "RA prefetch: TIMEOUT (eof=%d gen=%lu)", s_readahead_eof, (unsigned long)s_ra_gen);
}

static uint8_t  *s_mp3_inbuf = nullptr;   // MP3 decoder input window (PSRAM, 16KB)
static int16_t  *s_pcm_buf   = nullptr;   // MP3 decoded PCM output (PSRAM, ~4.6KB)
static uint8_t  *s_wav_buf   = nullptr;   // WAV write chunk buffer (PSRAM, 8KB)
static uint8_t  *s_id3_buf   = nullptr;   // ID3 tag frame buffer (PSRAM, 4KB)
#define ID3_BUF_SIZE 4096

// Album art — raw JPEG/PNG data extracted from APIC frame
static uint8_t  *s_art_buf   = nullptr;   // PSRAM buffer for album art
static size_t    s_art_size  = 0;          // size of data in s_art_buf
#define ART_BUF_MAX  (512 * 1024)          // max 512KB album art (compressed JPEG)

// ─── WAV Header ─────────────────────────────────────────────────────────────

#pragma pack(push, 1)
typedef struct {
    char     riff_header[4];   // "RIFF"
    uint32_t wav_size;
    char     wave_header[4];   // "WAVE"
    char     fmt_header[4];    // "fmt "
    uint32_t fmt_chunk_size;
    uint16_t audio_format;     // 1 = PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data_header[4];   // "data"
    uint32_t data_size;
} wav_header_t;
#pragma pack(pop)

// ─── ID3v2 Minimal Parser ──────────────────────────────────────────────────

static uint32_t id3_syncsafe(const uint8_t *b) {
    return ((uint32_t)b[0] << 21) | ((uint32_t)b[1] << 14) |
           ((uint32_t)b[2] << 7)  | (uint32_t)b[3];
}

static void id3_parse(const char *path, music_track_t *t) {
    FILE *f = fopen(path, "rb");
    if (!f) return;

    uint8_t hdr[10];
    if (fread(hdr, 1, 10, f) != 10 || memcmp(hdr, "ID3", 3) != 0) {
        fclose(f);
        return;
    }

    uint8_t ver_major = hdr[3];
    uint32_t tag_size = id3_syncsafe(hdr + 6);

    // Bulk-read the ID3 tag (up to ID3_BUF_SIZE) into PSRAM — one SD access.
    // TIT2/TPE1 are always near the start of the tag, well within 4KB.
    if (!s_id3_buf) { fclose(f); return; }
    size_t to_read = tag_size < ID3_BUF_SIZE ? tag_size : ID3_BUF_SIZE;
    size_t got = fread(s_id3_buf, 1, to_read, f);
    fclose(f);  // done with SD — everything parsed from PSRAM now

    if (got < 10) return;

    // Parse from buffer using pointer arithmetic — no seeks
    size_t pos = 0;

    // Skip extended header if present
    if (hdr[5] & 0x40 && pos + 4 <= got) {
        uint32_t ext_size = id3_syncsafe(s_id3_buf + pos);
        pos += ext_size;
    }

    // Scan frames
    while (pos + 10 <= got) {
        char frame_id[5] = {0};
        memcpy(frame_id, s_id3_buf + pos, 4);

        uint32_t frame_size;
        if (ver_major >= 4) {
            frame_size = id3_syncsafe(s_id3_buf + pos + 4);
        } else {
            frame_size = ((uint32_t)s_id3_buf[pos+4] << 24) | ((uint32_t)s_id3_buf[pos+5] << 16) |
                         ((uint32_t)s_id3_buf[pos+6] << 8)  | (uint32_t)s_id3_buf[pos+7];
        }
        pos += 10;  // past frame header

        if (frame_size == 0 || pos + frame_size > got) break;

        bool is_title  = (strcmp(frame_id, "TIT2") == 0);
        bool is_artist = (strcmp(frame_id, "TPE1") == 0);
        bool is_tlen   = (strcmp(frame_id, "TLEN") == 0);

        if (is_tlen) {
            // TLEN: duration in milliseconds as text string
            const uint8_t *frame_data = s_id3_buf + pos;
            uint8_t encoding = frame_data[0];
            // Simple ASCII/UTF-8 parse of the number (encoding 0 or 3)
            if (encoding == 0 || encoding == 3) {
                char dur_str[16] = {0};
                int len = (int)frame_size - 1;
                if (len > 15) len = 15;
                memcpy(dur_str, frame_data + 1, len);
                long ms = atol(dur_str);
                if (ms > 0) {
                    t->duration_s = ms / 1000.0;
                }
            }
        } else if (is_title || is_artist) {
            const uint8_t *frame_data = s_id3_buf + pos;
            uint8_t encoding = frame_data[0];
            const uint8_t *text_start = frame_data + 1;
            int text_len = (int)frame_size - 1;

            if (encoding == 1 && text_len >= 2) {
                // UTF-16: skip BOM, take every other byte (lossy but functional)
                const uint8_t *src = text_start;
                if (src[0] == 0xFF && src[1] == 0xFE) { src += 2; text_len -= 2; }
                else if (src[0] == 0xFE && src[1] == 0xFF) { src += 2; text_len -= 2; }

                char *dst = is_title ? t->title : t->artist;
                int dst_max = is_title ? MUSIC_TITLE_LEN - 1 : MUSIC_ARTIST_LEN - 1;
                int j = 0;
                for (int i = 0; i < text_len - 1 && j < dst_max; i += 2) {
                    uint8_t lo = src[i], hi = src[i + 1];
                    uint8_t ch = lo;
                    if (hi != 0) ch = '?';
                    if (ch >= 0x20) dst[j++] = ch;
                }
                dst[j] = '\0';
            } else {
                // ISO-8859-1 or UTF-8: copy directly
                if (text_len < 0) text_len = 0;
                char *dst = is_title ? t->title : t->artist;
                int max = is_title ? MUSIC_TITLE_LEN - 1 : MUSIC_ARTIST_LEN - 1;
                int len = text_len < max ? text_len : max;
                memcpy(dst, text_start, len);
                dst[len] = '\0';
            }
        }
        pos += frame_size;
    }
}

// ─── APIC Album Art Extraction ─────────────────────────────────────────────

// Extract embedded JPEG/PNG from ID3v2 APIC frame.
// Bulk-reads the entire ID3 tag into s_art_buf (one SD access), then
// parses from PSRAM. The image data ends up at the front of s_art_buf.
static void id3_extract_art(const char *path) {
    s_art_size = 0;  // clear previous art

    FILE *f = fopen(path, "rb");
    if (!f) return;

    uint8_t hdr[10];
    if (fread(hdr, 1, 10, f) != 10 || memcmp(hdr, "ID3", 3) != 0) {
        fclose(f);
        return;
    }

    uint8_t ver_major = hdr[3];
    uint32_t tag_size = id3_syncsafe(hdr + 6);

    if (!s_art_buf) { fclose(f); return; }

    // Bulk-read the entire ID3 tag into s_art_buf — one SD access.
    // If the tag is larger than our buffer, we read what fits and may
    // miss APIC frames near the end (unlikely — APIC is usually first large frame).
    size_t to_read = tag_size < ART_BUF_MAX ? tag_size : ART_BUF_MAX;
    if (tag_size > ART_BUF_MAX) {
        ESP_LOGW(TAG, "ID3 tag (%lu bytes) exceeds art buffer (%u bytes), truncating",
                 (unsigned long)tag_size, (unsigned)ART_BUF_MAX);
    }
    size_t got = fread(s_art_buf, 1, to_read, f);
    fclose(f);  // done with SD — everything parsed from PSRAM now

    if (got < 10) return;

    // Parse from buffer using pointer arithmetic — no seeks
    size_t pos = 0;

    // Skip extended header if present
    if (hdr[5] & 0x40 && pos + 4 <= got) {
        uint32_t ext_size = id3_syncsafe(s_art_buf + pos);
        pos += ext_size;
    }

    // Scan frames looking for APIC
    while (pos + 10 <= got) {
        char frame_id[5] = {0};
        memcpy(frame_id, s_art_buf + pos, 4);

        uint32_t frame_size;
        if (ver_major >= 4) {
            frame_size = id3_syncsafe(s_art_buf + pos + 4);
        } else {
            frame_size = ((uint32_t)s_art_buf[pos+4] << 24) | ((uint32_t)s_art_buf[pos+5] << 16) |
                         ((uint32_t)s_art_buf[pos+6] << 8)  | (uint32_t)s_art_buf[pos+7];
        }
        pos += 10;  // past frame header

        if (frame_size == 0) break;

        if (strcmp(frame_id, "APIC") == 0) {
            if (pos + frame_size > got) {
                ESP_LOGW(TAG, "APIC frame extends beyond buffered data (%lu + %lu > %lu)",
                         (unsigned long)pos, (unsigned long)frame_size, (unsigned long)got);
                break;
            }

            // Parse APIC structure directly from buffer at s_art_buf[pos]:
            //   [0]     text encoding
            //   [1..]   MIME type (null-terminated)
            //   [+1]    picture type byte
            //   [+1..]  description (null-terminated)
            //   [rest]  image data
            const uint8_t *apic = s_art_buf + pos;
            size_t apic_end = frame_size;
            size_t offset = 1;  // skip encoding byte

            // Skip MIME type
            while (offset < apic_end && apic[offset] != '\0') offset++;
            offset++;  // null terminator

            if (offset >= apic_end) break;
            offset++;  // picture type byte

            // Skip description
            uint8_t encoding = apic[0];
            if (encoding == 1 || encoding == 2) {
                // UTF-16: look for double null
                while (offset + 1 < apic_end) {
                    if (apic[offset] == 0 && apic[offset + 1] == 0) { offset += 2; break; }
                    offset += 2;
                }
            } else {
                while (offset < apic_end && apic[offset] != '\0') offset++;
                offset++;
            }

            if (offset >= apic_end) break;

            // Image data is at apic + offset, length = frame_size - offset.
            // Shift to front of s_art_buf so caller gets clean image data.
            size_t img_size = frame_size - offset;
            if (img_size > 0) {
                memmove(s_art_buf, apic + offset, img_size);
                s_art_size = img_size;
                ESP_LOGI(TAG, "Album art: %u bytes (offset %u in APIC)", (unsigned)img_size, (unsigned)offset);
            }
            break;  // only extract first APIC
        }

        // Skip non-APIC frame
        pos += frame_size;
    }
}

// ─── Filename Metadata Fallback ─────────────────────────────────────────────

static void metadata_from_filename(const char *path, music_track_t *t) {
    // Extract filename without extension
    const char *slash = strrchr(path, '/');
    const char *name = slash ? slash + 1 : path;

    // Copy to title, strip extension
    strncpy(t->title, name, MUSIC_TITLE_LEN - 1);
    t->title[MUSIC_TITLE_LEN - 1] = '\0';

    char *dot = strrchr(t->title, '.');
    if (dot) *dot = '\0';

    // Try to split "Artist - Title" format
    char *sep = strstr(t->title, " - ");
    if (sep) {
        *sep = '\0';
        strncpy(t->artist, t->title, MUSIC_ARTIST_LEN - 1);
        t->artist[MUSIC_ARTIST_LEN - 1] = '\0';
        memmove(t->title, sep + 3, strlen(sep + 3) + 1);
    }
}

// ─── File Extension Check ───────────────────────────────────────────────────

static music_format_t detect_format(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if (!dot) return MUSIC_FORMAT_UNKNOWN;

    char ext[8];
    int i = 0;
    for (const char *p = dot + 1; *p && i < 7; p++, i++)
        ext[i] = tolower((unsigned char)*p);
    ext[i] = '\0';

    if (strcmp(ext, "mp3") == 0) return MUSIC_FORMAT_MP3;
    if (strcmp(ext, "wav") == 0) return MUSIC_FORMAT_WAV;
    return MUSIC_FORMAT_UNKNOWN;
}

// ─── Init / Scan ────────────────────────────────────────────────────────────

void music_player_init(void) {
    if (!s_tracks) {
        s_tracks = (music_track_t *)heap_caps_calloc(
            MUSIC_MAX_TRACKS, sizeof(music_track_t), MALLOC_CAP_SPIRAM);
    }
    if (!s_mp3d) {
        s_mp3d = (mp3dec_t *)heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM);
    }
    if (!s_mp3_inbuf) {
        s_mp3_inbuf = (uint8_t *)heap_caps_malloc(MP3_INBUF_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (!s_pcm_buf) {
        s_pcm_buf = (int16_t *)heap_caps_malloc(
            MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    }
    if (!s_wav_buf) {
        s_wav_buf = (uint8_t *)heap_caps_malloc(WAV_READ_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (!s_readahead) {
        s_readahead = (uint8_t *)heap_caps_malloc(READAHEAD_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (!s_id3_buf) {
        s_id3_buf = (uint8_t *)heap_caps_malloc(ID3_BUF_SIZE, MALLOC_CAP_SPIRAM);
    }
    if (!s_art_buf) {
        s_art_buf = (uint8_t *)heap_caps_malloc(ART_BUF_MAX, MALLOC_CAP_SPIRAM);
    }

    if (!s_tracks || !s_mp3d || !s_mp3_inbuf || !s_pcm_buf || !s_wav_buf || !s_readahead || !s_id3_buf || !s_art_buf) {
        ESP_LOGE(TAG, "Failed to allocate music player buffers in PSRAM");
    }

    // Create readahead mutex and background fill task
    if (!s_ra_mutex) {
        s_ra_mutex = xSemaphoreCreateMutex();
    }
    if (!s_ra_task_hdl && s_readahead && s_ra_mutex) {
        xTaskCreateWithCaps(readahead_fill_task, "ra_fill", 4096,
                            nullptr, 2,  // low priority — below audio task
                            &s_ra_task_hdl, MALLOC_CAP_SPIRAM);
    }

    s_track_count = 0;
    s_current_idx = 0;
    s_state = MUSIC_STATE_STOPPED;

    ESP_LOGI(TAG, "Music player initialized (buffers: mp3d=%p inbuf=%p pcm=%p wav=%p readahead=%p[%uKB] id3=%p fill_task=%p)",
             s_mp3d, s_mp3_inbuf, s_pcm_buf, s_wav_buf, s_readahead, READAHEAD_SIZE / 1024, s_id3_buf, s_ra_task_hdl);
}

// Forward declarations
static void dir_update_mtime(void);

int music_player_scan(void) {
    if (!s_tracks) return 0;
    s_track_count = 0;

    DIR *dir = opendir(MUSIC_DIR);
    if (!dir) {
        ESP_LOGW(TAG, "Cannot open %s — no music directory", MUSIC_DIR);
        return 0;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL && s_track_count < MUSIC_MAX_TRACKS) {
        if (ent->d_type != DT_REG && ent->d_type != DT_UNKNOWN) continue;

        // Skip hidden files, macOS resource forks (._), and 8.3 short name aliases (~)
        const char *name = ent->d_name;
        if (name[0] == '.') continue;                          // .DS_Store, ._*
        if (name[0] == '_' && name[1] == (char)0xE5) continue; // 8.3 deleted entry marker
        if (strchr(name, '~') != NULL) continue;               // THEYD~3.MP3 etc.

        music_format_t fmt = detect_format(name);
        if (fmt == MUSIC_FORMAT_UNKNOWN) continue;

        music_track_t *t = &s_tracks[s_track_count];
        memset(t, 0, sizeof(*t));
        t->format = fmt;
        snprintf(t->path, MUSIC_PATH_LEN, "%s/%s", MUSIC_DIR, ent->d_name);

        // Try ID3v2 for MP3, fall back to filename
        if (fmt == MUSIC_FORMAT_MP3) {
            id3_parse(t->path, t);
        }
        if (t->title[0] == '\0') {
            metadata_from_filename(t->path, t);
        }

        ESP_LOGI(TAG, "Track %d: \"%s\" by \"%s\" [%s] %s",
                 s_track_count, t->title, t->artist,
                 fmt == MUSIC_FORMAT_MP3 ? "MP3" : "WAV", t->path);

        s_track_count++;
    }
    closedir(dir);

    ESP_LOGI(TAG, "Scanned %d tracks from %s", s_track_count, MUSIC_DIR);
    dir_update_mtime();
    return s_track_count;
}

// ─── Info / Accessors ───────────────────────────────────────────────────────

music_player_info_t music_player_get_info(void) {
    music_player_info_t info = {};
    info.state = s_state;
    info.track_index = s_current_idx;
    info.track_count = s_track_count;
    info.position_s = s_position_s;
    info.duration_s = s_duration_s;
    if (s_track_count > 0 && s_current_idx >= 0 && s_current_idx < s_track_count) {
        info.title = s_tracks[s_current_idx].title;
        info.artist = s_tracks[s_current_idx].artist;
    } else {
        info.title = "No mp3s found in";
        info.artist = "/sdcard/music/";
    }
    return info;
}

const music_track_t *music_player_get_track(int index) {
    if (index < 0 || index >= s_track_count) return nullptr;
    return &s_tracks[index];
}

int music_player_track_count(void) { return s_track_count; }

void music_player_set_track(int index) {
    if (index >= 0 && index < s_track_count)
        s_current_idx = index;
}

// ─── Control (non-blocking signals) ─────────────────────────────────────────

void music_player_stop(void) {
    s_stop_requested = true;
    s_pause_requested = false;
}

void music_player_pause(void) {
    if (s_state == MUSIC_STATE_PLAYING)
        s_pause_requested = true;
}

void music_player_resume(void) {
    s_pause_requested = false;
}

void music_player_next(void) {
    if (s_track_count == 0) return;
    s_current_idx = (s_current_idx + 1) % s_track_count;
    s_track_changed = true;
    s_stop_requested = true;  // exit current play loop
}

void music_player_prev(void) {
    if (s_track_count == 0) return;

    // If we're more than 3 seconds into the song, restart it.
    // Otherwise go to the previous track.
    if (s_position_s > 3.0) {
        s_seek_target = 0.0;
    } else {
        s_current_idx = (s_current_idx - 1 + s_track_count) % s_track_count;
        s_track_changed = true;
        s_stop_requested = true;
    }
}

void music_player_seek(double position_s) {
    s_seek_target = position_s;
}

bool music_player_track_changed(void) { return s_track_changed; }
void music_player_ack_track_change(void) { s_track_changed = false; }

// ─── MP3 Playback ───────────────────────────────────────────────────────────

static void play_mp3(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open: %s", path);
        return;
    }

    if (!s_mp3d || !s_mp3_inbuf || !s_pcm_buf) {
        ESP_LOGE(TAG, "MP3 buffers not allocated");
        fclose(f);
        return;
    }
    mp3dec_init(s_mp3d);

    uint8_t *inbuf = s_mp3_inbuf;   // persistent PSRAM decoder window (16KB)
    int16_t *pcm   = s_pcm_buf;     // persistent PSRAM buffer (~4.6KB)

    // Start read-ahead immediately — all reads go through PSRAM from here
    readahead_open(f);
    readahead_prefetch();

    // Skip ID3v2 tag if present (read through readahead)
    uint8_t id3hdr[10];
    if (readahead_read(id3hdr, 10) == 10 && memcmp(id3hdr, "ID3", 3) == 0) {
        uint32_t tag_size = id3_syncsafe(id3hdr + 6);
        readahead_seek(tag_size + 10, SEEK_SET);
    } else {
        readahead_seek(0, SEEK_SET);
    }

    // Get file size for duration estimation (SEEK_END falls through to fseek)
    long data_start = readahead_tell();
    readahead_seek(0, SEEK_END);
    long file_size = readahead_tell() - data_start;
    readahead_seek(data_start, SEEK_SET);
    readahead_prefetch();  // re-fill after seek

    size_t inbuf_len = 0;
    size_t inbuf_consumed = 0;
    double total_samples = 0;
    int sample_rate = 44100;  // will be updated from first frame
    int channels = 2;
    uint32_t ui_update_ms = 0;
    long bytes_decoded = 0;
    bool duration_locked = false;  // once we have a reliable duration, stop re-estimating

    s_position_s = 0;
    s_duration_s = 0;

    // Use pre-parsed TLEN from ID3 tag if available (exact duration in ms)
    if (s_current_idx >= 0 && s_current_idx < s_track_count &&
        s_tracks[s_current_idx].duration_s > 0) {
        s_duration_s = s_tracks[s_current_idx].duration_s;
        duration_locked = true;
        ESP_LOGI(TAG, "Duration from ID3 TLEN: %.1fs", s_duration_s);
    }

    while (!s_stop_requested) {
        // Handle pause
        while (s_pause_requested && !s_stop_requested) {
            s_state = MUSIC_STATE_PAUSED;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (s_stop_requested) break;
        s_state = MUSIC_STATE_PLAYING;

        // Handle seek — estimate byte position from running average bitrate
        if (s_seek_target >= 0 && s_duration_s > 0) {
            // Clamp to actual duration — prevents seeking past EOF
            if (s_seek_target > s_duration_s) s_seek_target = s_duration_s - 0.5;
            if (s_seek_target < 0) s_seek_target = 0;
            double frac = s_seek_target / s_duration_s;
            if (frac < 0) frac = 0;
            if (frac > 1) frac = 1;
            long seek_byte = data_start + (long)(frac * file_size);
            readahead_seek(seek_byte, SEEK_SET);
            readahead_prefetch();
            // Reset decoder state — new position may land mid-frame,
            // minimp3 will resync automatically
            mp3dec_init(s_mp3d);
            inbuf_len = 0;
            inbuf_consumed = 0;
            bytes_decoded = (long)(frac * file_size);
            total_samples = s_seek_target * sample_rate;
            s_position_s = s_seek_target;
            s_seek_target = -1.0;
        }

        // Refill decoder input window from read-ahead buffer (PSRAM → PSRAM, fast)
        if (inbuf_consumed > 0) {
            memmove(inbuf, inbuf + inbuf_consumed, inbuf_len - inbuf_consumed);
            inbuf_len -= inbuf_consumed;
            inbuf_consumed = 0;
        }
        if (inbuf_len < MP3_INBUF_SIZE) {
            size_t to_read = MP3_INBUF_SIZE - inbuf_len;
            size_t got = readahead_read(inbuf + inbuf_len, to_read);
            inbuf_len += got;
        }

        if (inbuf_len == 0) break;  // EOF

        // Decode one frame
        mp3dec_frame_info_t frame_info;
        int samples = mp3dec_decode_frame(s_mp3d, inbuf, (int)inbuf_len, pcm, &frame_info);

        if (frame_info.frame_bytes == 0) {
            // No valid frame found — skip one byte and retry
            inbuf_consumed = 1;
            continue;
        }

        inbuf_consumed = frame_info.frame_bytes;
        bytes_decoded += frame_info.frame_bytes;

        // On first frame, try to get exact duration from Xing/VBRI VBR header.
        // Must be outside samples>0 check — Xing frames often produce 0 samples.
        if (!duration_locked && bytes_decoded == frame_info.frame_bytes && frame_info.frame_bytes > 0) {
            // Scan the raw frame data for Xing/Info marker (within first 200 bytes)
            int scan_len = frame_info.frame_bytes < 200 ? frame_info.frame_bytes : 200;
            for (int i = 0; i < scan_len - 12; i++) {
                if ((memcmp(inbuf + i, "Xing", 4) == 0 || memcmp(inbuf + i, "Info", 4) == 0)) {
                    uint32_t flags = ((uint32_t)inbuf[i+4] << 24) | ((uint32_t)inbuf[i+5] << 16) |
                                     ((uint32_t)inbuf[i+6] << 8)  | (uint32_t)inbuf[i+7];
                    if (flags & 0x01) {  // total frames field present
                        uint32_t total_frames = ((uint32_t)inbuf[i+8] << 24) | ((uint32_t)inbuf[i+9] << 16) |
                                                ((uint32_t)inbuf[i+10] << 8)  | (uint32_t)inbuf[i+11];
                        // Determine samples per frame from MPEG version
                        int spf = (frame_info.hz >= 32000) ? 1152 : 576;
                        int rate = (frame_info.hz > 0) ? frame_info.hz : sample_rate;
                        s_duration_s = (double)total_frames * spf / rate;
                        duration_locked = true;
                        if (s_current_idx >= 0 && s_current_idx < s_track_count)
                            s_tracks[s_current_idx].duration_s = s_duration_s;
                        ESP_LOGI(TAG, "Duration from Xing: %.1fs (%lu frames, %d spf, %d Hz)",
                                 s_duration_s, (unsigned long)total_frames, spf, rate);
                    }
                    break;
                }
            }
        }

        if (samples > 0) {
            // Update rate info from decoded frame
            if (frame_info.hz > 0 && frame_info.hz != (unsigned)sample_rate) {
                sample_rate = frame_info.hz;
                music_set_output_rate(sample_rate);
            }
            if (frame_info.channels > 0) channels = frame_info.channels;

            // Fallback: running bitrate estimate (updates until stable)
            if (!duration_locked && frame_info.bitrate_kbps > 0 && total_samples > 0) {
                double decoded_secs = total_samples / (double)sample_rate;
                // Update every ~2s of decoded audio for smooth convergence
                if (decoded_secs >= 2.0) {
                    double avg_bitrate = (bytes_decoded * 8.0) / decoded_secs;
                    double est = (double)file_size / (avg_bitrate / 8.0);
                    // Only log on first estimate or when it changes significantly
                    if (s_duration_s == 0 || (est > s_duration_s * 1.05 || est < s_duration_s * 0.95)) {
                        ESP_LOGI(TAG, "Duration estimate: %.1fs (avg %dkbps, %.0fs decoded)",
                                 est, (int)(avg_bitrate / 1000.0), decoded_secs);
                    }
                    s_duration_s = est;
                    if (s_current_idx >= 0 && s_current_idx < s_track_count)
                        s_tracks[s_current_idx].duration_s = s_duration_s;
                    // Lock after 10s of audio — average is stable enough
                    if (decoded_secs >= 10.0) {
                        duration_locked = true;
                    }
                }
            }

            total_samples += samples;
            s_position_s = total_samples / (double)sample_rate;

            // Write decoded PCM to I2S
            size_t pcm_bytes = samples * channels * sizeof(int16_t);
            ES8311->write_data(pcm, pcm_bytes);
        }

        // Update UI every 500ms
        uint32_t now = esp_log_timestamp();
        if (now >= ui_update_ms) {
            ui_update_ms = now + 500;
        }
    }

    // When we reach natural EOF, we know the true duration — correct any estimate.
    // This fixes VBR tracks without Xing headers after first full playback.
    if (s_position_s > 0 && !s_stop_requested && s_seek_target < 0) {
        double true_dur = s_position_s;
        if (s_duration_s == 0 || true_dur < s_duration_s * 0.95 || true_dur > s_duration_s * 1.05) {
            ESP_LOGI(TAG, "Duration corrected at EOF: %.1fs (was %.1fs)", true_dur, s_duration_s);
        }
        s_duration_s = true_dur;
        if (s_current_idx >= 0 && s_current_idx < s_track_count)
            s_tracks[s_current_idx].duration_s = true_dur;
    }

    // Buffers are persistent — no free needed
    readahead_close();
}

static void play_wav(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "Cannot open: %s", path);
        return;
    }

    if (!s_wav_buf) {
        ESP_LOGE(TAG, "WAV buffer not allocated");
        fclose(f);
        return;
    }

    // Start read-ahead from beginning of file — header + data all through PSRAM
    readahead_open(f);
    readahead_prefetch();

    wav_header_t hdr;
    if (readahead_read(&hdr, sizeof(hdr)) != sizeof(hdr)) {
        ESP_LOGE(TAG, "Failed to read WAV header");
        readahead_close();
        return;
    }

    if (memcmp(hdr.riff_header, "RIFF", 4) != 0 ||
        memcmp(hdr.wave_header, "WAVE", 4) != 0) {
        ESP_LOGE(TAG, "Invalid WAV file: %s", path);
        readahead_close();
        return;
    }

    // Skip to "data" chunk (handle extra chunks between fmt and data)
    if (memcmp(hdr.data_header, "data", 4) != 0) {
        readahead_seek(12, SEEK_SET);  // after RIFF + WAVE
        uint8_t chunk[8];
        while (readahead_read(chunk, 8) == 8) {
            uint32_t chunk_size = *(uint32_t *)(chunk + 4);
            if (memcmp(chunk, "data", 4) == 0) {
                hdr.data_size = chunk_size;
                break;
            }
            readahead_seek(readahead_tell() + chunk_size, SEEK_SET);
        }
    }

    ESP_LOGI(TAG, "WAV: %ldHz %dch %dbit %ldbytes",
             (long)hdr.sample_rate, hdr.num_channels,
             hdr.bits_per_sample, (long)hdr.data_size);

    if (hdr.bits_per_sample != 16 || hdr.audio_format != 1) {
        ESP_LOGE(TAG, "Only 16-bit PCM WAV supported");
        readahead_close();
        return;
    }

    long data_start = readahead_tell();
    s_duration_s = (double)hdr.data_size /
                   (hdr.sample_rate * hdr.num_channels * (hdr.bits_per_sample / 8.0));
    s_position_s = 0;

    // Reconfigure I2S + codec for this WAV's sample rate
    music_set_output_rate(hdr.sample_rate);

    // Read-ahead is already primed from file open — data streaming starts from PSRAM
    uint8_t *buf = s_wav_buf;  // persistent PSRAM I2S write chunk (8KB)

    size_t bytes_remaining = hdr.data_size;
    uint32_t ui_update_ms = 0;
    double bytes_per_sec = hdr.sample_rate * hdr.num_channels * (hdr.bits_per_sample / 8.0);

    while (bytes_remaining > 0 && !s_stop_requested) {
        // Handle pause
        while (s_pause_requested && !s_stop_requested) {
            s_state = MUSIC_STATE_PAUSED;
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (s_stop_requested) break;
        s_state = MUSIC_STATE_PLAYING;

        // Handle seek — invalidate read-ahead, re-seek file
        if (s_seek_target >= 0) {
            long seek_offset = (long)(s_seek_target * bytes_per_sec);
            seek_offset -= seek_offset % hdr.block_align;
            if (seek_offset < 0) seek_offset = 0;
            if (seek_offset > (long)hdr.data_size) seek_offset = hdr.data_size;
            readahead_seek(data_start + seek_offset, SEEK_SET);
            readahead_prefetch();
            bytes_remaining = hdr.data_size - seek_offset;
            s_position_s = s_seek_target;
            s_seek_target = -1.0;
        }

        size_t to_read = WAV_READ_SIZE;
        if (to_read > bytes_remaining) to_read = bytes_remaining;

        size_t got = readahead_read(buf, to_read);
        if (got == 0) break;

        ES8311->write_data(buf, got);
        bytes_remaining -= got;

        s_position_s = (double)(hdr.data_size - bytes_remaining) / bytes_per_sec;

        // Update UI
        uint32_t now = esp_log_timestamp();
        if (now >= ui_update_ms) {
            ui_update_ms = now + 500;
        }
    }

    // Buffers are persistent — no free needed
    readahead_close();
}

// ─── Lightweight directory change detection ─────────────────────────────────

static time_t s_dir_mtime = 0;  // last known mtime of /sdcard/music/

/**
 * Check if /sdcard/music/ has been modified since last scan.
 * Uses directory mtime — catches adds, removes, renames, content swaps.
 * Single stat() call, no directory traversal.
 */
static bool dir_changed(void) {
    struct stat st;
    if (stat(MUSIC_DIR, &st) != 0) {
        // Directory gone (SD removed?) — treat as changed if we had tracks
        return s_track_count > 0;
    }
    if (st.st_mtime != s_dir_mtime) {
        return true;
    }
    return false;
}

/**
 * Update stored mtime after a successful scan.
 */
static void dir_update_mtime(void) {
    struct stat st;
    if (stat(MUSIC_DIR, &st) == 0) {
        s_dir_mtime = st.st_mtime;
    }
}

// ─── Main Play Entry Point ──────────────────────────────────────────────────

void music_player_play_blocking(void) {
    if (s_track_count == 0 || s_current_idx < 0 || s_current_idx >= s_track_count) {
        ESP_LOGW(TAG, "No tracks to play");
        return;
    }

    s_stop_requested = false;
    s_pause_requested = false;
    s_seek_target = -1.0;
    s_state = MUSIC_STATE_PLAYING;

    music_track_t *t = &s_tracks[s_current_idx];
    ESP_LOGI(TAG, "Playing [%d/%d]: \"%s\" by \"%s\" (%s)",
             s_current_idx + 1, s_track_count,
             t->title, t->artist,
             t->format == MUSIC_FORMAT_MP3 ? "MP3" : "WAV");

    // Check file is accessible before attempting playback
    FILE *test = fopen(t->path, "rb");
    if (!test) {
        ESP_LOGE(TAG, "Cannot open: %s (SD removed?)", t->path);
        s_state = MUSIC_STATE_STOPPED;
        s_position_s = 0;
        return;  // caller should not auto-retry
    }
    fclose(test);

    switch (t->format) {
        case MUSIC_FORMAT_MP3:
            play_mp3(t->path);
            break;
        case MUSIC_FORMAT_WAV:
            play_wav(t->path);
            break;
        default:
            ESP_LOGE(TAG, "Unknown format for %s", t->path);
            break;
    }

    // Check if directory was modified while playing (files added/removed/renamed)
    if (dir_changed()) {
        ESP_LOGI(TAG, "Directory modified, re-scanning");
        music_player_scan();
        if (s_current_idx >= s_track_count) s_current_idx = 0;
    }

    // Track finished or stopped
    if (!s_track_changed) {
        // Natural end of track — auto-advance
        if (s_track_count > 0)
            s_current_idx = (s_current_idx + 1) % s_track_count;
        ESP_LOGI(TAG, "Next track: %d/%d", s_current_idx + 1, s_track_count);
    }

    s_state = MUSIC_STATE_STOPPED;
    s_position_s = 0;
}

// ─── Album Art API ─────────────────────────────────────────────────────────

const uint8_t *music_player_get_album_art(size_t *out_size) {
    if (out_size) *out_size = s_art_size;
    return (s_art_size > 0) ? s_art_buf : NULL;
}

bool music_player_load_album_art(int track_index) {
    s_art_size = 0;
    if (!s_art_buf) return false;
    if (track_index < 0 || track_index >= s_track_count) return false;

    music_track_t *t = &s_tracks[track_index];
    if (t->format != MUSIC_FORMAT_MP3) return false;  // WAV has no embedded art

    id3_extract_art(t->path);
    if (s_art_size == 0) {
        ESP_LOGI(TAG, "No APIC frame found in \"%s\"", t->title);
    }
    return s_art_size > 0;
}
