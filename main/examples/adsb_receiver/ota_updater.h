// ota_updater.h — OTA firmware update via SD card
//
// Download flow:
//   1. HTTP GET ota.version.json → check version + get SHA256
//   2. HTTP GET ota.bin → stream to /sdcard/firmware/ota.bin
//   3. Verify SHA256 of downloaded binary
//   4. Write metadata to /sdcard/firmware/ota.meta.json
//   5. Set status.ready = true
//
// Flash flow:
//   1. Re-verify SHA256 of ota.bin on SD card
//   2. esp_ota_begin → read chunks from SD → esp_ota_write → esp_ota_end
//   3. esp_ota_set_boot_partition → reboot
//
// Rollback:
//   On boot, if NVS "ota_pending" is set but running version doesn't match,
//   bootloader rolled back. We log the failure and publish an MQTT ack.
//
// Part of ADS-B Scope — T-Display-P4.

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OTA_VERSION_URL  "https://adsb-scope.offx1.com/ota.version.json"
#define OTA_BIN_URL      "https://adsb-scope.offx1.com/ota.bin"
#define OTA_SD_DIR       "/sdcard/firmware"
#define OTA_SD_BIN       "/sdcard/firmware/ota.bin"
#define OTA_SD_META      "/sdcard/firmware/ota.meta.json"

// ─── Status ──────────────────────────────────────────────────────────────────

typedef struct {
    bool downloading;       // download task is running
    bool ready;             // verified binary on SD card, waiting for flash command
    char version[32];       // version of downloaded/ready binary
    int  progress_pct;      // download progress 0–100
    char error[80];         // last error message (empty = no error)
} ota_status_t;

// ─── Public API ──────────────────────────────────────────────────────────────

/// Initialize OTA subsystem. Call once from app_main after NVS is ready.
void ota_init(void);

/// Post-boot check: confirm new firmware or detect rollback.
/// Call from app_main after critical subsystems (WiFi, SD, MQTT) are up.
/// If rollback is detected, internal state is set for deferred MQTT ack.
void ota_post_boot_check(void);

/// Start async download of firmware to SD card.
/// @param force  If true, skip version check (re-download same version).
/// @return ESP_OK if download task started.
///         ESP_ERR_INVALID_STATE if already downloading.
esp_err_t ota_download_start(bool force);

/// Flash OTA binary from SD card to inactive OTA partition, then reboot.
/// This function does NOT return on success.
/// @param force  If true, flash even if downloaded version matches running.
/// @return Error code if flash failed (ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_CRC, etc.)
esp_err_t ota_flash_from_sd(bool force);

/// Get current OTA status (download progress, readiness, errors).
ota_status_t ota_get_status(void);

/// Get running firmware version string (from esp_app_desc).
const char *ota_running_version(void);

/// Check if a rollback ack is pending (call from MQTT on connect).
/// @param out_running  Buffer for current running version.
/// @param out_failed   Buffer for the version that was rolled back from.
/// @return true if there is a pending rollback to report.
bool ota_get_pending_rollback(char *out_running, size_t run_len,
                               char *out_failed, size_t fail_len);

/// Clear pending rollback flag (call after MQTT ack is published).
void ota_clear_pending_rollback(void);

#ifdef __cplusplus
}
#endif
