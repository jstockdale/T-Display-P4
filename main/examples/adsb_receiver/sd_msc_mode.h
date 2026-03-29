/**
 * sd_msc_mode.h — USB Mass Storage mode for SD card.
 *
 * Lifecycle:
 *   1. User taps "USB Storage" in Settings (or sends `msc` serial command)
 *   2. sd_msc_enter() is called:
 *      a. Flush + close ADS-B and Meshy SD logs
 *      b. Unmount SD from SPI, power-cycle via XL9535_SD_EN
 *      c. Remount in SDMMC 4-bit native mode (~20 MB/s)
 *      d. Initialize TinyUSB MSC device on USB Full-speed port
 *      e. Draw MSC status UI on screen
 *   3. User transfers files over USB
 *   4. User taps "Disconnect" on screen (or sends `cdc` serial command)
 *   5. sd_msc_exit() is called:
 *      a. Stop TinyUSB MSC
 *      b. Unmount SDMMC, remount SPI for normal operation
 *      c. Create fresh ADS-B + Meshy log files (new timestamps)
 *      d. Restore normal UI
 *
 * The USB Serial/JTAG console (printf) stays active throughout —
 * it's a separate hardware peripheral from the Full-speed USB port.
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ─── External functions we call (defined in other modules) ───────────────────

// SD card mount/unmount (main.cpp)
// SD card mount status (main.cpp)
extern "C" bool sd_is_mounted(void);
extern "C" void sd_safe_shutdown(void);
extern "C" bool sd_remount(void);
extern "C" bool sd_remount_sdmmc(void);

// ADS-B SD log (class_driver / adsb module)
extern "C" void sd_log_close(void);
extern "C" void sd_log_create_new(void);

// Meshy SD log (meshtastic_task.cpp)
extern "C" void meshy_sd_close(void);
extern "C" void meshy_sd_create_new(void);

// RTL-SDR USB host stop/restart (main.cpp)
// Stop releases the USB PHY so TinyUSB can claim it for MSC.
// Restart reinstalls USB host and class driver after MSC exits.
// DMA transfer buffers are never freed — they survive across cycles.
void rtlsdr_adsb_stop(void);
void rtlsdr_adsb_restart(void);

// TinyUSB MSC (tusb_msc_sd.h)
#include "tusb_msc_sd.h"

// ─── State ───────────────────────────────────────────────────────────────────

static const char *MSC_TAG = "MSC";

static volatile bool s_msc_active = false;

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * Enter USB Mass Storage mode.
 * Returns true if MSC mode started successfully.
 *
 * Call from LVGL Settings button handler or serial `msc` command.
 * The caller is responsible for updating the UI (drawing the MSC screen).
 */
static inline bool sd_msc_enter(void) {
    if (s_msc_active) {
        ESP_LOGW(MSC_TAG, "Already in MSC mode");
        return true;
    }

    ESP_LOGI(MSC_TAG, "=== Entering USB Mass Storage mode ===");

    // ── Step 1: Stop RTL-SDR USB host ──
    // Must stop FIRST — on_msg fires sd_log_aircraft() which does SPI writes.
    // If we unmount SD while on_msg is mid-write, the in-flight SPI DMA
    // overwrites freed PSRAM → total stack corruption.
    // Also releases the USB PHY so TinyUSB can claim it for MSC.
    ESP_LOGI(MSC_TAG, "Stopping RTL-SDR USB host...");
    rtlsdr_adsb_stop();

    // ── Step 2: Close all SD log files ──
    // Safe now — no more on_msg calls, no more sd_log_aircraft writes.
    ESP_LOGI(MSC_TAG, "Closing ADS-B log...");
    sd_log_close();

    ESP_LOGI(MSC_TAG, "Closing Meshy log...");
    meshy_sd_close();

    // ── Step 3: Unmount SD from SPI ──
    // Must unmount before power cycle — can't pull power with active FS
    ESP_LOGI(MSC_TAG, "Unmounting SD (SPI)...");
    sd_safe_shutdown();

    // ── Step 4: Power cycle + remount in SDMMC 4-bit mode ──
    ESP_LOGI(MSC_TAG, "Mounting SD (SDMMC 4-bit)...");
    if (!sd_remount_sdmmc()) {
        ESP_LOGE(MSC_TAG, "SDMMC mount failed — recovering");
        sd_remount();
        sd_log_create_new();
        meshy_sd_create_new();
        rtlsdr_adsb_restart();
        return false;
    }

    // ── Step 5: Start TinyUSB MSC ──
    ESP_LOGI(MSC_TAG, "Starting USB MSC...");
    if (!tusb_msc_start()) {
        ESP_LOGE(MSC_TAG, "TinyUSB MSC start failed — recovering");
        sd_safe_shutdown();
        sd_remount();
        sd_log_create_new();
        meshy_sd_create_new();
        rtlsdr_adsb_restart();
        return false;
    }

    s_msc_active = true;
    ESP_LOGI(MSC_TAG, "=== USB Mass Storage mode active (SDMMC 4-bit) ===");
    ESP_LOGI(MSC_TAG, "ADS-B receiver and logging paused. SD card available to host.");
    return true;
}

/**
 * Exit USB Mass Storage mode and return to normal operation.
 * Creates fresh log files with new timestamps.
 *
 * Call from the on-screen Disconnect button or serial `cdc` command.
 */
static inline void sd_msc_exit(void) {
    if (!s_msc_active) {
        ESP_LOGW(MSC_TAG, "Not in MSC mode");
        return;
    }

    ESP_LOGI(MSC_TAG, "=== Exiting USB Mass Storage mode ===");

    // Log final transfer stats
    uint64_t read_bytes  = tusb_msc_bytes_read();
    uint64_t write_bytes = tusb_msc_bytes_written();
    ESP_LOGI(MSC_TAG, "Transfer stats: %" PRIu64 " bytes read, %" PRIu64 " bytes written by host",
             read_bytes, write_bytes);

    // ── Step 1: Stop TinyUSB MSC ──
    ESP_LOGI(MSC_TAG, "Stopping USB MSC...");
    tusb_msc_stop();

    // ── Step 2: Unmount SD from SDMMC ──
    ESP_LOGI(MSC_TAG, "Unmounting SD (SDMMC)...");
    sd_safe_shutdown();

    // ── Step 3: Remount SD in SPI mode (normal operation) ──
    ESP_LOGI(MSC_TAG, "Mounting SD (SPI)...");
    if (!sd_remount()) {
        ESP_LOGE(MSC_TAG, "SPI remount failed — SD card logging unavailable");
    }

    // ── Step 4: Create fresh log files with new timestamps ──
    if (sd_is_mounted()) {
        ESP_LOGI(MSC_TAG, "Creating fresh ADS-B log...");
        sd_log_create_new();

        ESP_LOGI(MSC_TAG, "Creating fresh Meshy log...");
        meshy_sd_create_new();
    }

    // ── Step 5: Restart RTL-SDR USB host ──
    // Reinstalls USB host library (reclaims PHY) and class driver.
    // adsb_reader_task will be spawned when the RTL-SDR device reconnects.
    ESP_LOGI(MSC_TAG, "Restarting RTL-SDR USB host...");
    rtlsdr_adsb_restart();

    s_msc_active = false;
    ESP_LOGI(MSC_TAG, "=== Normal operation resumed ===");
}

/**
 * Check if we're currently in USB MSC mode.
 */
static inline bool sd_msc_is_active(void) {
    return s_msc_active;
}

/**
 * Get transfer stats for the UI activity indicator.
 * Only valid while MSC mode is active.
 */
static inline void sd_msc_get_stats(uint64_t *bytes_read, uint64_t *bytes_written) {
    if (s_msc_active) {
        *bytes_read  = tusb_msc_bytes_read();
        *bytes_written = tusb_msc_bytes_written();
    } else {
        *bytes_read = 0;
        *bytes_written = 0;
    }
}

// ─── Serial Console Commands ─────────────────────────────────────────────────
//
// Add to your serial command parser:
//
//   if (strcmp(cmd, "msc") == 0) {
//       if (sd_msc_enter()) {
//           printf("[MSC] USB Mass Storage mode active. Type 'cdc' to exit.\n");
//           // Trigger MSC UI draw
//       } else {
//           printf("[MSC] Failed to enter MSC mode.\n");
//       }
//   }
//   else if (strcmp(cmd, "cdc") == 0) {
//       sd_msc_exit();
//       printf("[MSC] Normal operation resumed.\n");
//       // Restore normal UI
//   }
//
// ─── LVGL Settings Integration ───────────────────────────────────────────────
//
// Add a "USB Storage" button to the Settings panel.
// On tap:
//   1. Show confirmation dialog:
//      "Enter USB Storage mode? ADS-B and Meshy logging will stop."
//      [Cancel] [Enter USB Storage]
//   2. On confirm: sd_msc_enter() → draw MSC status screen
//   3. MSC status screen shows:
//      - USB icon (large, centered)
//      - "USB Storage Active"
//      - "↑ XXX.X MB read" / "↓ XXX.X MB written" (updated every 500ms via lv_timer)
//      - Activity indicator (spinning or pulsing when bytes change)
//      - [Disconnect] button → sd_msc_exit() → restore Settings screen
