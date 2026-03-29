/**
 * tusb_msc_sd.h — TinyUSB Mass Storage Class bridge to SD card.
 *
 * Exposes the mounted SD card as a USB Mass Storage device on the
 * ESP32-P4's USB Full-speed port (port 15 on T-Display-P4).
 *
 * USB port allocation:
 *   - USB Serial/JTAG (port 16): printf console + WebSerial — always on
 *   - USB 2.0 HS OTG (port 17/18): RTL-SDR host — always on
 *   - USB Full-speed (port 15): TinyUSB MSC device — only during MSC mode
 *
 * The FS and HS ports are independent controllers, so MSC can run
 * simultaneously with the RTL-SDR host and the serial console.
 *
 * Prerequisites:
 *   - SD card must be mounted (SPI or SDMMC — sdmmc_card_t* is valid)
 *   - For best performance, mount in SDMMC 4-bit mode before calling start()
 *   - Add to idf_component.yml:
 *       espressif/esp_tinyusb:
 *         version: ">=0.0.1"
 *
 * sdkconfig:
 *   CONFIG_TINYUSB_ENABLED=y
 *   CONFIG_TINYUSB_MSC_ENABLED=y
 *   CONFIG_TINYUSB_MSC_BUFSIZE=4096
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "sdmmc_cmd.h"

// ─── External: SD card handle from main.cpp ──────────────────────────────────

extern sdmmc_card_t *sd_card_handle;   // set by Sd_Spi_Init or Sdmmc_Init

// ─── Requires esp_tinyusb component + sdkconfig ─────────────────────────────
// sdkconfig.defaults must include:
//   CONFIG_TINYUSB_ENABLED=y
//   CONFIG_TINYUSB_MSC_ENABLED=y
// Then run: idf.py reconfigure

#include <atomic>
#include "tinyusb.h"

// ─── State ───────────────────────────────────────────────────────────────────

static const char *TUSB_TAG = "TUSB_MSC";

static volatile bool s_tusb_msc_running = false;
static std::atomic<uint64_t> s_bytes_read{0};
static std::atomic<uint64_t> s_bytes_written{0};

// ─── TinyUSB MSC Callbacks ───────────────────────────────────────────────────
//
// These are the raw TinyUSB callbacks that the MSC class driver invokes.
// They bridge USB SCSI block commands to the ESP-IDF sdmmc driver.
//
// Note: These run in the TinyUSB task context, which has its own stack.
// sdmmc_read_sectors / sdmmc_write_sectors are thread-safe.

/**
 * Invoked when host sends SCSI READ10 command.
 * Read `bufsize` bytes from `lba` into `buffer`.
 * Returns number of bytes read, or -1 on error.
 */
extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba,
                                      uint32_t offset, void *buffer,
                                      uint32_t bufsize)
{
    (void)lun;

    if (!sd_card_handle) return -1;

    uint32_t sector_size = sd_card_handle->csd.sector_size;
    if (sector_size == 0) sector_size = 512;

    // offset = byte offset from start of READ10 command
    // Adjust LBA for multi-chunk transfers
    uint32_t adj_lba = lba + (offset / sector_size);
    uint32_t count = bufsize / sector_size;
    if (count == 0) return 0;

    esp_err_t err = sdmmc_read_sectors(sd_card_handle, buffer, adj_lba, count);
    if (err != ESP_OK) {
        ESP_LOGE(TUSB_TAG, "Read error at LBA %lu: %s", (unsigned long)adj_lba, esp_err_to_name(err));
        return -1;
    }

    s_bytes_read.fetch_add(bufsize, std::memory_order_relaxed);
    return (int32_t)bufsize;
}

/**
 * Invoked when host sends SCSI WRITE10 command.
 * Write `bufsize` bytes from `buffer` to `lba`.
 * Returns number of bytes written, or -1 on error.
 */
extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba,
                                       uint32_t offset, uint8_t *buffer,
                                       uint32_t bufsize)
{
    (void)lun;

    if (!sd_card_handle) return -1;

    uint32_t sector_size = sd_card_handle->csd.sector_size;
    if (sector_size == 0) sector_size = 512;

    // offset = byte offset from start of WRITE10 command
    // Adjust LBA for multi-chunk transfers
    uint32_t adj_lba = lba + (offset / sector_size);
    uint32_t count = bufsize / sector_size;
    if (count == 0) return 0;

    esp_err_t err = sdmmc_write_sectors(sd_card_handle, buffer, adj_lba, count);
    if (err != ESP_OK) {
        ESP_LOGE(TUSB_TAG, "Write error at LBA %lu: %s", (unsigned long)adj_lba, esp_err_to_name(err));
        return -1;
    }

    s_bytes_written.fetch_add(bufsize, std::memory_order_relaxed);
    return (int32_t)bufsize;
}

/**
 * Invoked when host sends SCSI INQUIRY command.
 * Application fills vendor/product/revision strings.
 */
extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                                    uint8_t product_id[16],
                                    uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id,   "ADS-B   ", 8);
    memcpy(product_id,  "Scope SD Card   ", 16);
    memcpy(product_rev, "0100", 4);
}

/**
 * Invoked when host sends TEST UNIT READY command.
 * Return true if the device is ready.
 */
extern "C" bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return sd_card_handle != NULL;
}

/**
 * Invoked when host sends READ CAPACITY command.
 * Returns block count and block size.
 */
extern "C" void tud_msc_capacity_cb(uint8_t lun,
                                     uint32_t *block_count,
                                     uint16_t *block_size)
{
    (void)lun;
    if (!sd_card_handle) {
        *block_count = 0;
        *block_size  = 512;
        return;
    }

    uint32_t sector_size = sd_card_handle->csd.sector_size;
    if (sector_size == 0) sector_size = 512;

    // sdmmc_card_t stores capacity in KB in real_freq_khz... no.
    // The total number of sectors is available from the CSD register.
    // For SDHC/SDXC: capacity = (c_size + 1) * 512K bytes
    // But the sdmmc driver gives us card->csd.capacity in KB.
    uint64_t total_bytes = (uint64_t)sd_card_handle->csd.capacity * 1024ULL;
    *block_count = (uint32_t)(total_bytes / sector_size);
    *block_size  = (uint16_t)sector_size;

    ESP_LOGI(TUSB_TAG, "Capacity: %lu blocks x %u bytes = %llu MB",
             (unsigned long)*block_count, *block_size,
             (unsigned long long)(total_bytes / (1024 * 1024)));
}

/**
 * Invoked when host sends SCSI command that is not in the built-in list.
 * Return 0 for unsupported commands.
 */
extern "C" int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                                    void *buffer, uint16_t bufsize)
{
    (void)lun;
    (void)buffer;
    (void)bufsize;

    // Log unknown SCSI commands for debugging
    ESP_LOGD(TUSB_TAG, "Unknown SCSI cmd: 0x%02x", scsi_cmd[0]);

    // Set sense data: invalid command operation code
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

/**
 * Invoked when host performs a safe-eject.
 * We could trigger sd_msc_exit() here, but it's safer to let the
 * user press the Disconnect button — the host eject just means
 * "I'm done writing, you can drop the device."
 */
extern "C" bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                                       bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;

    if (load_eject) {
        if (!start) {
            // Host ejected — signal the UI
            ESP_LOGI(TUSB_TAG, "Host ejected SD card");
            // Could set a flag here for the UI to auto-disconnect:
            // s_host_ejected = true;
        }
    }
    return true;
}

/**
 * Invoked to check if the device is writable.
 * Return true to allow writes.
 */
extern "C" bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;
    return true;  // SD card is read-write
}

// ─── USB Device Descriptors ──────────────────────────────────────────────────

// String descriptor indices
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_MSC,
};

static const tusb_desc_device_t s_msc_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,     // defined by interface
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,   // Espressif VID
    .idProduct          = 0x4002,   // Custom PID for MSC mode
    .bcdDevice          = 0x0100,
    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,
    .bNumConfigurations = 1,
};

static const char *s_msc_string_descriptors[] = {
    [STRID_LANGID]       = "\x09\x04",      // English (US)
    [STRID_MANUFACTURER] = "ADS-B Scope",
    [STRID_PRODUCT]      = "SD Card",
    [STRID_SERIAL]       = "000001",
    [STRID_MSC]          = "ADS-B Scope SD Card",
};

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * Initialize TinyUSB in MSC mode on the Full-speed USB port.
 * SD card must already be mounted.
 * Returns true on success.
 */
extern "C" bool tusb_msc_start(void) {
    if (s_tusb_msc_running) {
        ESP_LOGW(TUSB_TAG, "Already running");
        return true;
    }

    if (!sd_card_handle) {
        ESP_LOGE(TUSB_TAG, "No SD card mounted");
        return false;
    }

    // Reset stats
    s_bytes_read.store(0, std::memory_order_relaxed);
    s_bytes_written.store(0, std::memory_order_relaxed);

    // Configure TinyUSB
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor     = &s_msc_device_descriptor,
        .string_descriptor     = s_msc_string_descriptors,
        .string_descriptor_count = sizeof(s_msc_string_descriptors) / sizeof(s_msc_string_descriptors[0]),
        .external_phy          = false,
        .configuration_descriptor = NULL,  // use default MSC config descriptor
        .self_powered          = false,
        .vbus_monitor_io       = -1,
    };

    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TUSB_TAG, "TinyUSB install failed: %s", esp_err_to_name(err));
        return false;
    }

    s_tusb_msc_running = true;
    ESP_LOGI(TUSB_TAG, "USB MSC started — SD card exposed to host");
    ESP_LOGI(TUSB_TAG, "Card: %s, %.1f GB",
             sd_card_handle->cid.name,
             (float)sd_card_handle->csd.capacity / (1024.0f * 1024.0f));

    return true;
}

/**
 * Stop TinyUSB MSC.
 * After this, the Full-speed port is idle.
 */
extern "C" void tusb_msc_stop(void) {
    if (!s_tusb_msc_running) return;

    ESP_LOGI(TUSB_TAG, "Stopping USB MSC...");
    ESP_LOGI(TUSB_TAG, "Final stats: %" PRIu64 " bytes read, %" PRIu64 " bytes written",
             s_bytes_read.load(), s_bytes_written.load());

    tinyusb_driver_uninstall();

    s_tusb_msc_running = false;
    ESP_LOGI(TUSB_TAG, "USB MSC stopped");
}

/**
 * Get cumulative transfer stats.
 */
extern "C" uint64_t tusb_msc_bytes_read(void) {
    return s_bytes_read.load(std::memory_order_relaxed);
}

extern "C" uint64_t tusb_msc_bytes_written(void) {
    return s_bytes_written.load(std::memory_order_relaxed);
}
