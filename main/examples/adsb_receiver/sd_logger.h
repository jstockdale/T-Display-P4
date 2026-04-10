/**
 * sd_logger.h — Shared SD card CSV log writer for ADS-B Scope.
 *
 * Provides a reusable "channel" that manages a persistent FILE handle,
 * fwrite+fflush+fsync flush cycle, error counting, and coordinated
 * SD card recovery (unmount + power-cycle + remount with backoff).
 *
 * Usage:
 *   static sd_log_ch_t my_ch;
 *   sd_log_ch_init(&my_ch, "MYTAG");
 *   sd_log_ch_open(&my_ch, "/sdcard/logs/foo.csv", "col1,col2\n");
 *   ...
 *   sd_log_ch_flush(&my_ch, buf, len);  // periodic
 *   ...
 *   sd_log_ch_close(&my_ch);            // shutdown
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// ── Per-channel state ────────────────────────────────────────────────

typedef struct {
    char        filename[64];
    FILE       *fp;                 // persistent handle (open between flushes)
    int         flush_fail_count;   // consecutive failures
    bool        initialized;        // true after successful open
    const char *tag;                // ESP_LOG tag (e.g., "CLASS", "MESHY")
} sd_log_ch_t;

// ── Channel API ──────────────────────────────────────────────────────

// Zero-init channel and set log tag.  Call once at startup.
void sd_log_ch_init(sd_log_ch_t *ch, const char *tag);

// Create a log file at `filename`, write `csv_header`, keep handle open.
// Creates /sdcard/logs/ directory if it doesn't exist.
// Returns true on success (ch->initialized set).
bool sd_log_ch_open(sd_log_ch_t *ch, const char *filename,
                    const char *csv_header);

// Reopen a previous log file in append mode (no header written).
// Uses ch->filename from a prior sd_log_ch_open().  Returns false if
// the file doesn't exist or filename was never set.
bool sd_log_ch_reopen(sd_log_ch_t *ch);

// Flush `len` bytes from `buf` to the persistent file handle.
// On repeated failure (5×), marks channel uninitialized and triggers
// shared SD recovery (unmount).  Timer output only covers I/O.
void sd_log_ch_flush(sd_log_ch_t *ch, const char *buf, int len);

// Close the log file and mark channel uninitialized.
void sd_log_ch_close(sd_log_ch_t *ch);

// Flush pending data, close handle, rename file, reopen with new name.
// On ENOENT (source file gone), updates filename and marks channel
// uninitialized — caller should create a fresh file with proper header.
// Returns true if the rename succeeded.
bool sd_log_ch_rename(sd_log_ch_t *ch, const char *new_filename,
                      const char *buf, int len);

// ── Shared recovery ──────────────────────────────────────────────────

// Attempt SD card recovery (unmount + power-cycle + remount).
// Coordinates across all channels via shared backoff state.
// Call from any task when a channel fails.
// Returns true if the card is now mounted and usable.
bool sd_log_try_recovery(const char *caller_tag);

// Reset recovery backoff (call after a successful open proves the card works).
void sd_log_recovery_reset(void);

// ── SPI bus health watchdog ──────────────────────────────────────────
// Returns true if a flush detected FAT corruption (dirty/IO-error flag
// set after a successful write) or if the FAT is so corrupt that new
// files cannot be created.  All further writes are blocked.
bool sd_log_bus_failed(void);

// Set the bus failure flag with a reason string.  Called by callers
// when they detect "mounted but can't create files" (zombie FAT).
void sd_log_bus_fail(const char *reason);

// Clear the bus failure flag.  Call when the user manually reformats
// the card and runs 'mount' to retry.
void sd_log_bus_reset(void);

// ── Flush stagger API ────────────────────────────────────────────────
// Channel IDs for flush stagger coordination
#define SD_FLUSH_CH_ADSB   0
#define SD_FLUSH_CH_MESHY  1

// Record that a channel just completed a flush.
// Call AFTER a successful sd_log_ch_flush().
void sd_log_record_flush(int channel);

// Returns the timestamp (us) of the OTHER channel's last flush.
// Used by each writer to stagger: "flush when 5s since other flushed,
// or 10s since self flushed, or buffer critical."
int64_t sd_log_other_flush_time(int channel);

#ifdef __cplusplus
}
#endif

#endif // SD_LOGGER_H
