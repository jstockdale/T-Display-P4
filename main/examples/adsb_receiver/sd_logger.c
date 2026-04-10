/**
 * sd_logger.c — Shared SD card CSV log writer for ADS-B Scope.
 *
 * All file I/O is serialized through sd_io_take/sd_io_give (SPI bus mutex).
 * Recovery (unmount + power-cycle + remount) is shared across channels
 * with exponential backoff so multiple writers don't fight each other.
 *
 * AXI DMA fence:
 * On ESP32-P4, SDHOST IDMAC (ESP-Hosted WiFi over SDIO) and GDMA-AXI
 * (GP-SPI SD card writes) share the AXI interconnect.  A silicon-level
 * bus arbitration bug can cause DMA writes to land at wrong addresses,
 * destroying FAT structures.  The DMA fence is acquired automatically
 * inside sd_io_take() (main.cpp) so ALL callers are protected.
 * See: https://github.com/espressif/esp-idf/issues/18235
 */

#include "sd_logger.h"

#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ── External SD infrastructure (defined in main.cpp) ─────────────────

extern bool sd_io_take(uint32_t timeout_ms);
extern void sd_io_give(void);
extern void sd_safe_shutdown(void);
extern bool sd_remount(void);
extern bool sd_is_mounted(void);
extern bool sd_check_dirty_flag(void);
extern bool sd_is_user_unmounted(void);

// ── AXI DMA fence — weak stubs for linker fallback ─────────────────
// The real DMA fence is now acquired inside sd_io_take()/sd_io_give()
// in main.cpp, so ALL SD callers are automatically protected.
// These weak stubs provide a safe no-op when ESP-Hosted is not linked
// (WiFi disabled) — no SDIO DMA to contend with.

bool __attribute__((weak)) esp_hosted_sdio_dma_lock(uint32_t timeout_ms)  { (void)timeout_ms; return true; }
void __attribute__((weak)) esp_hosted_sdio_dma_unlock(void) { }

// DMA fence timeout for metadata ops (open/close/rename).
// These are infrequent, so we proceed even if the lock times out –
// a brief contention window is better than failing the entire operation.

// ── Shared recovery state ────────────────────────────────────────────

static int64_t s_last_recovery_us   = 0;
static int     s_recovery_fail_count = 0;

// ── SPI bus health watchdog ──────────────────────────────────────────
// If the FAT dirty flag appears after a flush, or the FAT is so corrupt
// that new files can't be created, stop all writes to prevent further
// damage.  Cleared only by user running 'mount'.
static volatile bool s_sd_bus_failed = false;
static uint32_t s_dirty_check_count = 0;

// ── Flush stagger: per-channel timestamps so writers alternate ────────

#define SD_FLUSH_CH_ADSB   0
#define SD_FLUSH_CH_MESHY  1
#define SD_FLUSH_CH_COUNT  2

static int64_t s_flush_ts[SD_FLUSH_CH_COUNT] = {0, 0};

// ── Channel API ──────────────────────────────────────────────────────

void sd_log_ch_init(sd_log_ch_t *ch, const char *tag)
{
    memset(ch, 0, sizeof(*ch));
    ch->tag = tag;
}

bool sd_log_ch_open(sd_log_ch_t *ch, const char *filename,
                    const char *csv_header)
{
    if (ch->initialized) return true;  // already open

    if (!sd_io_take(5000)) return false;

    // Fence: fopen/fputs/fflush/fsync all trigger SPI DMA

    mkdir("/sdcard/logs", 0755);  // idempotent

    FILE *f = fopen(filename, "w");
    if (!f) {
        int e = errno;
        sd_io_give();
        ESP_LOGW(ch->tag, "Failed to open SD log: %s (errno=%d: %s) "
                 "[DMA free=%u largest=%u, internal=%u]",
            filename, e, strerror(e),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        return false;
    }

    if (csv_header) {
        fputs(csv_header, f);
    }
    fflush(f);
    fsync(fileno(f));


    // Update state only AFTER the file was successfully created.
    // This avoids leaving a stale filename on failure.
    strncpy(ch->filename, filename, sizeof(ch->filename) - 1);
    ch->filename[sizeof(ch->filename) - 1] = '\0';
    ch->fp = f;
    sd_io_give();

    ch->initialized = true;
    ch->flush_fail_count = 0;
    sd_log_recovery_reset();  // card is working — clear backoff
    ESP_LOGI(ch->tag, "SD log: %s", ch->filename);
    return true;
}

bool sd_log_ch_reopen(sd_log_ch_t *ch)
{
    if (ch->initialized) return true;   // already open
    if (ch->filename[0] == '\0') return false;  // no previous file

    if (!sd_io_take(5000)) return false;

    // Fence: fopen triggers SPI DMA for directory/FAT reads

    FILE *f = fopen(ch->filename, "a");


    if (!f) {
        int e = errno;
        sd_io_give();
        ESP_LOGW(ch->tag, "SD reopen failed: %s (errno=%d: %s)",
            ch->filename, e, strerror(e));
        return false;
    }

    ch->fp = f;
    sd_io_give();

    ch->initialized = true;
    ch->flush_fail_count = 0;
    sd_log_recovery_reset();
    ESP_LOGI(ch->tag, "SD log reopened: %s", ch->filename);
    return true;
}

void sd_log_ch_flush(sd_log_ch_t *ch, const char *buf, int len)
{
    if (!ch->initialized || len <= 0) return;

    // SPI bus watchdog: if a prior flush detected FAT corruption, refuse
    // all further writes to prevent additional damage.
    if (s_sd_bus_failed) return;

    if (!sd_io_take(5000)) return;

    // Reopen handle if it was closed after a prior error.
    // Fenced: fopen triggers SPI DMA for directory/FAT reads.
    if (!ch->fp) {
        ch->fp = fopen(ch->filename, "a");

        if (!ch->fp) {
            int e = errno;
            sd_io_give();

            ch->flush_fail_count++;
            ESP_LOGW(ch->tag, "SD flush: cannot reopen %s (%d consecutive, "
                     "errno=%d: %s) [DMA free=%u largest=%u, internal=%u]",
                ch->filename, ch->flush_fail_count, e, strerror(e),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

            if (ch->flush_fail_count >= 5) {
                ESP_LOGW(ch->tag, "SD card unresponsive after %d failures "
                         "— unmounting", ch->flush_fail_count);
                ch->initialized = false;
                ch->flush_fail_count = 0;
                if (sd_is_mounted()) {
                    sd_safe_shutdown();
                }
            }
            return;
        }
    }

    // ── Write + flush (sd_io held — includes DMA fence) ─────────

    int64_t t0 = esp_timer_get_time();

    size_t total_written = fwrite(buf, 1, (size_t)len, ch->fp);
    fflush(ch->fp);
    fsync(fileno(ch->fp));


    int64_t elapsed_us = esp_timer_get_time() - t0;
    // ── End fenced write ─────────────────────────────────────────

    if ((int)total_written != len) {
        ESP_LOGW(ch->tag, "SD flush: partial write (%d/%d), closing handle "
                 "[DMA free=%u largest=%u, internal=%u]",
            (int)total_written, len,
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

        // Fence the error-path fclose too
        fclose(ch->fp);

        ch->fp = NULL;
        ch->flush_fail_count++;
        sd_io_give();

        if (ch->flush_fail_count >= 5) {
            ESP_LOGW(ch->tag, "SD card unresponsive after %d failures "
                     "— unmounting", ch->flush_fail_count);
            ch->initialized = false;
            ch->flush_fail_count = 0;
            if (sd_is_mounted()) {
                sd_safe_shutdown();
            }
        }
        return;
    }

    // ── Dirty flag watchdog ──────────────────────────────────────────
    // After a successful write+fsync, check if the FAT dirty/IO-error
    // flags appeared.  If so, the SPI bus is corrupting sector addresses
    // and data is landing in the FAT region.  Stop all writes immediately.
    s_dirty_check_count++;
    if (sd_check_dirty_flag()) {
        ESP_LOGE(ch->tag,
            "*** SPI BUS INTEGRITY FAILURE ***  "
            "FAT dirty/IO-error flag set after flush #%lu.  "
            "Stopping all SD writes to prevent further corruption.  "
            "Card may need reformatting.",
            (unsigned long)s_dirty_check_count);
        s_sd_bus_failed = true;

        // Fence the error-path fclose too
        fclose(ch->fp);

        ch->fp = NULL;
        ch->initialized = false;
        sd_io_give();
        return;
    }

    sd_io_give();

    ch->flush_fail_count = 0;
    ESP_LOGI(ch->tag, "SD flush: %d bytes in %lld ms",
             len, elapsed_us / 1000);
}

void sd_log_record_flush(int channel) {
    if (channel >= 0 && channel < SD_FLUSH_CH_COUNT)
        s_flush_ts[channel] = esp_timer_get_time();
}

int64_t sd_log_other_flush_time(int channel) {
    int other = (channel == SD_FLUSH_CH_ADSB) ? SD_FLUSH_CH_MESHY : SD_FLUSH_CH_ADSB;
    return s_flush_ts[other];
}

void sd_log_ch_close(sd_log_ch_t *ch)
{
    if (!ch->initialized) return;

    // Caller should flush their buffer before calling close.
    // We just close the file handle.
    if (ch->fp) {
        if (sd_io_take(5000)) {
            fclose(ch->fp);
            sd_io_give();
        }
        ch->fp = NULL;
    }
    ch->initialized = false;
    ch->flush_fail_count = 0;
    ESP_LOGI(ch->tag, "SD log closed: %s", ch->filename);
}

bool sd_log_ch_rename(sd_log_ch_t *ch, const char *new_filename,
                      const char *buf, int len)
{
    if (!ch->initialized) return false;

    // Flush any pending data under the old name
    if (len > 0) {
        sd_log_ch_flush(ch, buf, len);
    }

    if (!sd_io_take(5000)) return false;

    // Fence the entire close + rename + reopen sequence.
    // All three operations trigger SPI DMA.

    // Close persistent handle before rename
    if (ch->fp) {
        fclose(ch->fp);
        ch->fp = NULL;
    }

    bool ok = (rename(ch->filename, new_filename) == 0);
    if (ok) {
        ESP_LOGI(ch->tag, "SD log renamed: %s → %s", ch->filename, new_filename);
        strncpy(ch->filename, new_filename, sizeof(ch->filename) - 1);
        ch->filename[sizeof(ch->filename) - 1] = '\0';
        // Reopen renamed file in append mode
        ch->fp = fopen(ch->filename, "a");
        if (!ch->fp) {
            ESP_LOGE(ch->tag, "SD log reopen failed after rename");
            ch->initialized = false;
        }
    } else {
        int e = errno;
        ESP_LOGW(ch->tag, "SD log rename failed: %s → %s (errno=%d: %s)",
                 ch->filename, new_filename, e, strerror(e));
        if (e == ENOENT) {
            // Source file gone (SD failure / corruption destroyed it).
            // Update filename to the new name so the _boot retry check
            // stops firing.  Mark channel uninitialized — caller should
            // detect this and create a fresh file with proper header.
            ESP_LOGW(ch->tag, "Source file gone — caller must create new log");
            strncpy(ch->filename, new_filename, sizeof(ch->filename) - 1);
            ch->filename[sizeof(ch->filename) - 1] = '\0';
        }
        ch->initialized = false;
    }

    sd_io_give();
    return ok;
}

// ── Shared recovery ──────────────────────────────────────────────────

extern bool sd_is_user_unmounted(void);

bool sd_log_try_recovery(const char *caller_tag)
{
    if (sd_is_mounted()) return true;  // card is fine

    // Don't auto-remount if the user explicitly ran 'unmount'
    if (sd_is_user_unmounted()) return false;

    // Don't attempt recovery if SPI bus integrity failure was detected —
    // further writes risk more FAT corruption.  User must reformat or
    // replace the card, then use 'mount' to clear this flag.
    if (s_sd_bus_failed) return false;

    int64_t now_us = esp_timer_get_time();

    // Backoff: 30s → 60s → 120s → 300s max
    int64_t interval_us;
    if (s_recovery_fail_count <= 0)      interval_us =  30000000LL;
    else if (s_recovery_fail_count == 1) interval_us =  60000000LL;
    else if (s_recovery_fail_count == 2) interval_us = 120000000LL;
    else                                 interval_us = 300000000LL;

    if (now_us - s_last_recovery_us < interval_us) return false;
    s_last_recovery_us = now_us;

    ESP_LOGW(caller_tag, "SD recovery attempt %d — remounting...",
             s_recovery_fail_count + 1);

    if (sd_is_mounted()) {
        sd_safe_shutdown();
    }

    if (sd_remount()) {
        ESP_LOGI(caller_tag, "SD recovery succeeded");
        // Don't reset fail count here — reset when a file is actually opened
        return true;
    }

    s_recovery_fail_count++;
    int next_s = (s_recovery_fail_count <= 1) ? 60
               : (s_recovery_fail_count == 2) ? 120 : 300;
    ESP_LOGW(caller_tag, "SD recovery failed (%d attempts) — next retry in %ds",
             s_recovery_fail_count, next_s);
    return false;
}

void sd_log_recovery_reset(void)
{
    s_recovery_fail_count = 0;
}

bool sd_log_bus_failed(void)
{
    return s_sd_bus_failed;
}

void sd_log_bus_fail(const char *reason)
{
    if (!s_sd_bus_failed) {
        ESP_LOGE("SD", "*** SD BUS FAILED *** %s", reason);
    }
    s_sd_bus_failed = true;
}

void sd_log_bus_reset(void)
{
    if (s_sd_bus_failed) {
        ESP_LOGW("SD", "SPI bus failure flag cleared — user requested retry");
    }
    s_sd_bus_failed = false;
    s_dirty_check_count = 0;
}
