// ota_updater.c — OTA firmware update via SD card
//
// See ota_updater.h for design overview and public API.
//
// Part of ADS-B Scope — T-Display-P4.
// ESP-IDF v5.4.1 on ESP32-P4.

#include "ota_updater.h"

#include <string.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"

#define TAG "OTA"

#define OTA_NVS_NAMESPACE  "ota"
#define OTA_NVS_PENDING    "pending_ver"
#define OTA_DL_BUF_SIZE    (16 * 1024)    // 16KB download buffer (PSRAM)
#define OTA_FLASH_BUF_SIZE (4096)         // 4KB flash write buffer
#define OTA_TASK_STACK     (8192)
#define OTA_TASK_PRIORITY  (3)

// ISRG Root X1 PEM — shared with mqtt_feeder.c (removed static there).
// Both OTA HTTPS downloads and MQTT TLS use Let's Encrypt certs.
extern const char ISRG_ROOT_X1_PEM[];

// SD card I/O mutex — defined in main.cpp, shared with class_driver and meshy
extern bool sd_io_take(uint32_t timeout_ms);
extern void sd_io_give(void);
extern bool sd_is_mounted(void);

// ─── State ───────────────────────────────────────────────────────────────────

static nvs_handle_t s_nvs = 0;
static ota_status_t s_status = {0};
static TaskHandle_t s_download_task = NULL;

static bool s_rollback_pending = false;
static char s_rollback_failed_ver[32] = {0};

// ─── Helpers ─────────────────────────────────────────────────────────────────

static void set_error(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_status.error, sizeof(s_status.error), fmt, ap);
    va_end(ap);
    ESP_LOGW(TAG, "%s", s_status.error);
}

static void clear_error(void) {
    s_status.error[0] = '\0';
}

/// Convert 32-byte SHA256 hash to 64-char hex string.
static void sha256_to_hex(const unsigned char *hash, char *hex, size_t hex_size) {
    if (hex_size < 65) return;
    for (int i = 0; i < 32; i++) {
        snprintf(hex + i * 2, 3, "%02x", hash[i]);
    }
}

/// Compute SHA256 of a file on SD card. Returns hex string.
static esp_err_t sha256_file(const char *path, char *hex_out, size_t hex_size) {
    if (!sd_io_take(5000)) return ESP_ERR_TIMEOUT;
    FILE *f = fopen(path, "rb");
    sd_io_give();
    if (!f) return ESP_ERR_NOT_FOUND;

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);  // 0 = SHA-256

    char *buf = heap_caps_malloc(OTA_DL_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!buf) {
        if (sd_io_take(200)) { fclose(f); sd_io_give(); }
        else fclose(f);
        mbedtls_sha256_free(&ctx);
        return ESP_ERR_NO_MEM;
    }

    size_t n;
    while (1) {
        if (!sd_io_take(5000)) break;
        n = fread(buf, 1, OTA_DL_BUF_SIZE, f);
        sd_io_give();
        if (n == 0) break;
        mbedtls_sha256_update(&ctx, (unsigned char *)buf, n);
    }
    if (sd_io_take(200)) { fclose(f); sd_io_give(); }
    else fclose(f);
    heap_caps_free(buf);

    unsigned char hash[32];
    mbedtls_sha256_finish(&ctx, hash);
    mbedtls_sha256_free(&ctx);

    sha256_to_hex(hash, hex_out, hex_size);
    return ESP_OK;
}

/// Ensure /sdcard/firmware/ directory exists.
static void ensure_firmware_dir(void) {
    struct stat st;
    if (stat(OTA_SD_DIR, &st) != 0) {
        mkdir(OTA_SD_DIR, 0755);
    }
}

/// Write metadata JSON to SD card.
static esp_err_t write_meta(const char *version, const char *sha256) {
    if (!sd_io_take(5000)) return ESP_ERR_TIMEOUT;
    FILE *f = fopen(OTA_SD_META, "w");
    if (!f) { sd_io_give(); return ESP_FAIL; }
    fprintf(f, "{\"version\":\"%s\",\"sha256\":\"%s\"}", version, sha256);
    fclose(f);
    sd_io_give();
    return ESP_OK;
}

/// Read metadata JSON from SD card.
static esp_err_t read_meta(char *version, size_t ver_size, char *sha256, size_t sha_size) {
    if (!sd_io_take(5000)) return ESP_ERR_TIMEOUT;
    FILE *f = fopen(OTA_SD_META, "r");
    if (!f) { sd_io_give(); return ESP_ERR_NOT_FOUND; }

    char buf[256];
    size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    sd_io_give();
    buf[n] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) return ESP_ERR_INVALID_ARG;

    cJSON *jver = cJSON_GetObjectItem(root, "version");
    cJSON *jsha = cJSON_GetObjectItem(root, "sha256");
    if (!jver || !jsha || !cJSON_IsString(jver) || !cJSON_IsString(jsha)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(version, jver->valuestring, ver_size - 1);
    version[ver_size - 1] = '\0';
    strncpy(sha256, jsha->valuestring, sha_size - 1);
    sha256[sha_size - 1] = '\0';

    cJSON_Delete(root);
    return ESP_OK;
}

// ─── HTTP Download ───────────────────────────────────────────────────────────

/// Download a small JSON file into a malloc'd string. Caller frees.
static char *http_get_json(const char *url) {
    esp_http_client_config_t cfg = {
        .url = url,
        .cert_pem = ISRG_ROOT_X1_PEM,
        .timeout_ms = 15000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return NULL;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP %d for %s", status, url);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    // Cap at 4KB for safety
    int alloc_size = (content_length > 0 && content_length < 4096) ? content_length + 1 : 4096;
    char *buf = malloc(alloc_size);
    if (!buf) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int total = 0;
    int n;
    while ((n = esp_http_client_read(client, buf + total, alloc_size - total - 1)) > 0) {
        total += n;
        if (total >= alloc_size - 1) break;
    }
    buf[total] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return buf;
}

/// Download a large file to SD card, computing SHA256 as we go.
/// Returns ESP_OK if download succeeded and SHA256 matches expected.
static esp_err_t http_download_to_sd(const char *url, const char *path,
                                      const char *expected_sha) {
    esp_http_client_config_t cfg = {
        .url = url,
        .cert_pem = ISRG_ROOT_X1_PEM,
        .timeout_ms = 30000,
        .buffer_size = OTA_DL_BUF_SIZE,
        .buffer_size_tx = 1024,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) { set_error("HTTP init failed"); return ESP_FAIL; }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        set_error("HTTP connect failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        set_error("HTTP %d for ota.bin", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Downloading %d bytes to %s", content_length, path);

    // Allocate download buffer in PSRAM
    char *buf = heap_caps_malloc(OTA_DL_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!buf) {
        set_error("No PSRAM for download buffer");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    // Open output file
    ensure_firmware_dir();
    if (!sd_io_take(5000)) {
        set_error("SD mutex timeout");
        heap_caps_free(buf);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(path, "wb");
    sd_io_give();
    if (!f) {
        set_error("Cannot create %s (errno=%d)", path, errno);
        heap_caps_free(buf);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    // Stream download: HTTP read → SHA256 update → SD write
    mbedtls_sha256_context sha_ctx;
    mbedtls_sha256_init(&sha_ctx);
    mbedtls_sha256_starts(&sha_ctx, 0);

    int total = 0;
    bool write_error = false;
    int n;

    while ((n = esp_http_client_read(client, buf, OTA_DL_BUF_SIZE)) > 0) {
        mbedtls_sha256_update(&sha_ctx, (unsigned char *)buf, n);

        // Write to SD under mutex
        if (sd_io_take(5000)) {
            size_t written = fwrite(buf, 1, n, f);
            sd_io_give();
            if ((int)written != n) {
                set_error("SD write failed (%d/%d)", (int)written, n);
                write_error = true;
                break;
            }
        } else {
            set_error("SD mutex timeout during download");
            write_error = true;
            break;
        }

        total += n;
        if (content_length > 0) {
            s_status.progress_pct = (total * 100) / content_length;
        }

        // Yield to let other tasks run (ADSB decode, Meshy RX)
        vTaskDelay(1);
    }

    // Flush and close
    if (sd_io_take(5000)) {
        fflush(f);
        fsync(fileno(f));
        fclose(f);
        sd_io_give();
    } else {
        fclose(f);
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    heap_caps_free(buf);

    if (write_error) {
        if (sd_io_take(200)) { remove(path); sd_io_give(); }
        mbedtls_sha256_free(&sha_ctx);
        return ESP_FAIL;
    }

    if (content_length > 0 && total != content_length) {
        set_error("Incomplete download: %d/%d bytes", total, content_length);
        if (sd_io_take(200)) { remove(path); sd_io_give(); }
        mbedtls_sha256_free(&sha_ctx);
        return ESP_FAIL;
    }

    // Verify SHA256
    unsigned char hash[32];
    mbedtls_sha256_finish(&sha_ctx, hash);
    mbedtls_sha256_free(&sha_ctx);

    char hex[65];
    sha256_to_hex(hash, hex, sizeof(hex));

    if (strcmp(hex, expected_sha) != 0) {
        set_error("SHA256 mismatch: expected %.16s..., got %.16s...",
                  expected_sha, hex);
        if (sd_io_take(200)) { remove(path); sd_io_give(); }
        return ESP_ERR_INVALID_CRC;
    }

    ESP_LOGI(TAG, "Download complete: %d bytes, SHA256 verified", total);
    return ESP_OK;
}

// ─── Download Task ───────────────────────────────────────────────────────────

static void download_task(void *arg) {
    bool force = (bool)(intptr_t)arg;

    // MQTT ack function — extern to avoid header dependency
    extern void mqtt_feeder_publish_ack(const char *, const char *,
                                         const char *, const char *);

    clear_error();
    s_status.downloading = true;
    s_status.ready = false;
    s_status.progress_pct = 0;

    const char *dl_cmd = force ? "firmware_download_force" : "firmware_download";

    // 1. Fetch version manifest
    ESP_LOGI(TAG, "Checking %s", OTA_VERSION_URL);
    char *json_str = http_get_json(OTA_VERSION_URL);
    if (!json_str) {
        set_error("Failed to fetch ota.version.json");
        goto done;
    }

    // 2. Parse version + sha256
    cJSON *root = cJSON_Parse(json_str);
    free(json_str);
    if (!root) {
        set_error("Invalid JSON in ota.version.json");
        goto done;
    }

    cJSON *jver = cJSON_GetObjectItem(root, "version");
    cJSON *jsha = cJSON_GetObjectItem(root, "sha256");
    if (!jver || !jsha || !cJSON_IsString(jver) || !cJSON_IsString(jsha)) {
        set_error("Missing version or sha256 in manifest");
        cJSON_Delete(root);
        goto done;
    }

    const char *remote_ver = jver->valuestring;
    const char *remote_sha = jsha->valuestring;

    // 3. Version check
    const char *running = ota_running_version();
    if (!force && strcmp(remote_ver, running) == 0) {
        set_error("Already running %s", running);
        cJSON_Delete(root);
        goto done;
    }

    ESP_LOGI(TAG, "Update available: %s → %s", running, remote_ver);

    // Copy strings before cJSON_Delete
    char ver_buf[32], sha_buf[65];
    strncpy(ver_buf, remote_ver, sizeof(ver_buf) - 1);
    ver_buf[sizeof(ver_buf) - 1] = '\0';
    strncpy(sha_buf, remote_sha, sizeof(sha_buf) - 1);
    sha_buf[sizeof(sha_buf) - 1] = '\0';
    cJSON_Delete(root);

    // 4. Check SD card
    if (!sd_is_mounted()) {
        set_error("SD card not mounted");
        goto done;
    }

    // 5. Download binary
    esp_err_t err = http_download_to_sd(OTA_BIN_URL, OTA_SD_BIN, sha_buf);
    if (err != ESP_OK) {
        // Error already set by http_download_to_sd
        goto done;
    }

    // 6. Write metadata
    err = write_meta(ver_buf, sha_buf);
    if (err != ESP_OK) {
        set_error("Failed to write metadata");
        if (sd_io_take(200)) { remove(OTA_SD_BIN); sd_io_give(); }
        goto done;
    }

    // 7. Success
    strncpy(s_status.version, ver_buf, sizeof(s_status.version) - 1);
    s_status.ready = true;
    s_status.progress_pct = 100;
    clear_error();
    ESP_LOGI(TAG, "Firmware %s ready on SD card", ver_buf);
    mqtt_feeder_publish_ack(dl_cmd, "ok", NULL, ver_buf);

done:
    if (!s_status.ready && s_status.error[0]) {
        // Download failed — publish error ack
        mqtt_feeder_publish_ack(dl_cmd, "error", s_status.error, NULL);
    }
    s_status.downloading = false;
    s_download_task = NULL;
    vTaskDelete(NULL);
}

// ─── Public API ──────────────────────────────────────────────────────────────

void ota_init(void) {
    esp_err_t err = nvs_open(OTA_NVS_NAMESPACE, NVS_READWRITE, &s_nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
    }

    // Check if there's already a ready firmware on SD card
    char ver[32], sha[65];
    if (read_meta(ver, sizeof(ver), sha, sizeof(sha)) == ESP_OK) {
        struct stat st;
        if (stat(OTA_SD_BIN, &st) == 0 && st.st_size > 0) {
            strncpy(s_status.version, ver, sizeof(s_status.version) - 1);
            s_status.ready = true;
            ESP_LOGI(TAG, "Found ready firmware on SD: %s", ver);
        }
    }
}

void ota_post_boot_check(void) {
    if (!s_nvs) return;

    // Read pending version from NVS
    char pending[32] = {0};
    size_t len = sizeof(pending);
    esp_err_t err = nvs_get_str(s_nvs, OTA_NVS_PENDING, pending, &len);
    if (err != ESP_OK || pending[0] == '\0') {
        return;  // No pending OTA — normal boot
    }

    const char *running = ota_running_version();

    if (strcmp(pending, running) == 0) {
        // OTA succeeded — running version matches what we flashed
        esp_ota_mark_app_valid_cancel_rollback();
        nvs_erase_key(s_nvs, OTA_NVS_PENDING);
        nvs_commit(s_nvs);
        ESP_LOGI(TAG, "OTA confirmed: now running %s", running);

        // Clean up SD card
        if (sd_io_take(200)) {
            remove(OTA_SD_BIN);
            remove(OTA_SD_META);
            sd_io_give();
        }
    } else {
        // Rollback happened — running version differs from pending
        ESP_LOGW(TAG, "OTA ROLLBACK: running %s, attempted %s", running, pending);

        // Create failure marker on SD + clean up
        if (sd_io_take(5000)) {
            char failed_path[80];
            snprintf(failed_path, sizeof(failed_path), OTA_SD_DIR "/%s.failed", pending);
            FILE *f = fopen(failed_path, "w");
            if (f) {
                fprintf(f, "{\"attempted\":\"%s\",\"reverted_to\":\"%s\"}", pending, running);
                fclose(f);
            }
            remove(OTA_SD_BIN);
            remove(OTA_SD_META);
            sd_io_give();
        }
        nvs_erase_key(s_nvs, OTA_NVS_PENDING);
        nvs_commit(s_nvs);

        // Defer MQTT ack until connected
        s_rollback_pending = true;
        strncpy(s_rollback_failed_ver, pending, sizeof(s_rollback_failed_ver) - 1);
        s_rollback_failed_ver[sizeof(s_rollback_failed_ver) - 1] = '\0';
    }
}

esp_err_t ota_download_start(bool force) {
    if (s_status.downloading) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create download task on PSRAM stack
    BaseType_t ret = xTaskCreate(
        download_task,
        "ota_dl",
        OTA_TASK_STACK,
        (void *)(intptr_t)force,
        OTA_TASK_PRIORITY,
        &s_download_task
    );

    if (ret != pdPASS) {
        set_error("Failed to create download task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t ota_flash_from_sd(bool force) {
    // 1. Read metadata
    char ver[32], expected_sha[65];
    if (read_meta(ver, sizeof(ver), expected_sha, sizeof(expected_sha)) != ESP_OK) {
        ESP_LOGE(TAG, "No firmware metadata on SD card");
        return ESP_ERR_NOT_FOUND;
    }

    // 2. Version check
    const char *running = ota_running_version();
    if (!force && strcmp(ver, running) == 0) {
        ESP_LOGW(TAG, "Firmware %s is already running (use force to reinstall)", ver);
        return ESP_ERR_INVALID_STATE;
    }

    // 3. Verify file exists
    struct stat st;
    if (stat(OTA_SD_BIN, &st) != 0) {
        ESP_LOGE(TAG, "Firmware file not found: %s", OTA_SD_BIN);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Firmware file: %s (%ld bytes)", OTA_SD_BIN, st.st_size);

    // 4. Re-verify SHA256 before flashing
    ESP_LOGI(TAG, "Verifying SHA256...");
    char actual_sha[65];
    esp_err_t err = sha256_file(OTA_SD_BIN, actual_sha, sizeof(actual_sha));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHA256 computation failed");
        return err;
    }
    if (strcmp(actual_sha, expected_sha) != 0) {
        ESP_LOGE(TAG, "SHA256 mismatch — firmware file corrupted");
        ESP_LOGE(TAG, "  expected: %.16s...", expected_sha);
        ESP_LOGE(TAG, "  actual:   %.16s...", actual_sha);
        return ESP_ERR_INVALID_CRC;
    }
    ESP_LOGI(TAG, "SHA256 verified");

    // 5. Save pending version to NVS (for rollback detection)
    if (s_nvs) {
        nvs_set_str(s_nvs, OTA_NVS_PENDING, ver);
        nvs_commit(s_nvs);
    }

    // 6. Find next OTA partition
    const esp_partition_t *running_part = esp_ota_get_running_partition();
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(running_part);
    if (!update_part) {
        ESP_LOGE(TAG, "No OTA update partition available");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "Flashing to partition: %s (offset 0x%lx, size %lu)",
             update_part->label, update_part->address, update_part->size);

    if ((size_t)st.st_size > update_part->size) {
        ESP_LOGE(TAG, "Firmware too large: %ld > %lu", st.st_size, update_part->size);
        return ESP_ERR_INVALID_SIZE;
    }

    // 7. Begin OTA
    esp_ota_handle_t handle = 0;
    err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return err;
    }

    // 8. Read from SD and write to OTA partition
    if (!sd_io_take(5000)) {
        ESP_LOGE(TAG, "SD busy, cannot read firmware file");
        esp_ota_abort(handle);
        return ESP_ERR_TIMEOUT;
    }
    FILE *f = fopen(OTA_SD_BIN, "rb");
    sd_io_give();
    if (!f) {
        ESP_LOGE(TAG, "Cannot open %s for reading", OTA_SD_BIN);
        esp_ota_abort(handle);
        return ESP_FAIL;
    }

    char *buf = heap_caps_malloc(OTA_FLASH_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!buf) {
        if (sd_io_take(200)) { fclose(f); sd_io_give(); }
        else fclose(f);
        esp_ota_abort(handle);
        return ESP_ERR_NO_MEM;
    }

    int total = 0;
    size_t n;
    while (1) {
        // Hold sd_io only for fread — release before esp_ota_write (flash I/O)
        if (!sd_io_take(5000)) break;
        n = fread(buf, 1, OTA_FLASH_BUF_SIZE, f);
        sd_io_give();
        if (n == 0) break;

        err = esp_ota_write(handle, buf, n);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed at offset %d: %s", total, esp_err_to_name(err));
            heap_caps_free(buf);
            if (sd_io_take(200)) { fclose(f); sd_io_give(); }
            else fclose(f);
            esp_ota_abort(handle);
            return err;
        }
        total += n;

        // Progress log every 512KB
        if (total % (512 * 1024) == 0) {
            ESP_LOGI(TAG, "Flashing... %d / %ld bytes (%d%%)",
                     total, st.st_size, (int)((int64_t)total * 100 / st.st_size));
        }
    }
    if (sd_io_take(200)) { fclose(f); sd_io_give(); }
    else fclose(f);
    heap_caps_free(buf);

    ESP_LOGI(TAG, "Wrote %d bytes to OTA partition", total);

    // 9. Finalize OTA
    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "OTA image validation failed — image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        }
        return err;
    }

    // 10. Set boot partition
    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "OTA complete: %s → %s. Rebooting in 2s...", running, ver);

    // Publish success ack before rebooting
    extern void mqtt_feeder_publish_ack(const char *, const char *,
                                         const char *, const char *);
    mqtt_feeder_publish_ack("firmware_update", "ok", NULL, ver);

    // Give MQTT time to publish the ack
    vTaskDelay(pdMS_TO_TICKS(2000));

    esp_restart();

    // Never reached
    return ESP_OK;
}

ota_status_t ota_get_status(void) {
    return s_status;
}

const char *ota_running_version(void) {
    const esp_app_desc_t *app = esp_app_get_description();
    return app->version;
}

bool ota_get_pending_rollback(char *out_running, size_t run_len,
                               char *out_failed, size_t fail_len) {
    if (!s_rollback_pending) return false;

    if (out_running && run_len > 0) {
        strncpy(out_running, ota_running_version(), run_len - 1);
        out_running[run_len - 1] = '\0';
    }
    if (out_failed && fail_len > 0) {
        strncpy(out_failed, s_rollback_failed_ver, fail_len - 1);
        out_failed[fail_len - 1] = '\0';
    }
    return true;
}

void ota_clear_pending_rollback(void) {
    s_rollback_pending = false;
    s_rollback_failed_ver[0] = '\0';
}
