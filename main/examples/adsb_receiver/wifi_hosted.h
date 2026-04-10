/**
 * wifi_hosted.h — WiFi connectivity via ESP-Hosted (C6 over SDIO).
 *
 * The ESP32-C6 runs the ESP-Hosted "network_adapter" slave firmware,
 * communicating with the P4 over SDMMC Slot 1 (GPIOs 14-19).
 * This module wraps the standard ESP-IDF WiFi APIs — esp_hosted +
 * esp_wifi_remote make the transport transparent.
 *
 * Architecture:
 *   - C6 is held in reset (XL9535_ESP32C6_EN = LOW) until wifi_hosted_init()
 *   - ESP-Hosted manages the SDIO transport automatically
 *   - Standard esp_wifi_* APIs work once initialized
 *   - SNTP time sync starts automatically on WiFi connect
 *
 * Prerequisites:
 *   - SD card must be on SPI (not SDMMC) to avoid DMA conflict (Issue #17889)
 *   - C6 must be pre-flashed with ESP-Hosted SPI slave firmware
 *   - idf_component.yml must include esp_hosted and esp_wifi_remote
 *
 * sdkconfig entries (add to sdkconfig.defaults):
 *   CONFIG_ESP_WIFI_REMOTE_ENABLED=y
 *   CONFIG_ESP_HOSTED_ENABLED=y
 *   CONFIG_ESP_HOSTED_TRANSPORT_SDIO=y
 *   CONFIG_ESP_HOSTED_SDIO_SLOT=1
 *   CONFIG_ESP_HOSTED_SDIO_CLK_GPIO=18
 *   CONFIG_ESP_HOSTED_SDIO_CMD_GPIO=19
 *   CONFIG_ESP_HOSTED_SDIO_D0_GPIO=14
 *   CONFIG_ESP_HOSTED_SDIO_D1_GPIO=15
 *   CONFIG_ESP_HOSTED_SDIO_D2_GPIO=16
 *   CONFIG_ESP_HOSTED_SDIO_D3_GPIO=17
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_heap_caps.h"

// Time manager — must be included before event handler uses esp_sntp / NTP functions
#include "time_manager.h"
#include "serial_console.h"  // for serial_console_print, log_format_timestamp

// XL9535 GPIO expander pin control (from your project)
// These must be provided by the caller or linked from main.cpp
extern "C" void xl9535_c6_enable(bool enable);   // drive XL9535_ESP32C6_EN
extern "C" void xl9535_c6_wakeup(void);           // pulse XL9535_ESP32C6_WAKE_UP

static const char *WIFI_TAG = "WIFI";

// ─── Configuration ───────────────────────────────────────────────────────────

// WiFi credentials — stored in NVS in production, hardcoded for dev
#ifndef WIFI_SSID
#define WIFI_SSID     ""       // set via menuconfig or NVS
#endif
#ifndef WIFI_PASS
#define WIFI_PASS     ""
#endif

// NTP config lives in time_manager.h / device_settings

// Event group bits
#define WIFI_CONNECTED_BIT   BIT0
#define WIFI_FAIL_BIT        BIT1

// Max reconnect attempts before giving up (-1 = infinite)
#define WIFI_MAX_RETRIES     10

// ─── State ───────────────────────────────────────────────────────────────────

static EventGroupHandle_t s_wifi_event_group = NULL;
static int s_wifi_retry_count = 0;
static volatile bool s_wifi_initialized = false;
static volatile bool s_wifi_connected = false;
static volatile bool s_wifi_paused = false;
static esp_netif_t *s_sta_netif = NULL;

// WiFi credentials (runtime configurable)
static char s_wifi_ssid[33] = WIFI_SSID;
static char s_wifi_pass[65] = WIFI_PASS;

// ─── Event Handler ───────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(WIFI_TAG, "STA started, connecting...");
                esp_wifi_connect();
                break;

            case WIFI_EVENT_STA_DISCONNECTED: {
                s_wifi_connected = false;
                wifi_event_sta_disconnected_t *d =
                    (wifi_event_sta_disconnected_t *)event_data;
                ESP_LOGW(WIFI_TAG, "Disconnected (reason=%d)", d->reason);

                if (s_wifi_paused) break; // don't reconnect if intentionally paused

                if (WIFI_MAX_RETRIES < 0 || s_wifi_retry_count < WIFI_MAX_RETRIES) {
                    s_wifi_retry_count++;
                    ESP_LOGI(WIFI_TAG, "Reconnect attempt %d/%d",
                             s_wifi_retry_count, WIFI_MAX_RETRIES);
                    vTaskDelay(pdMS_TO_TICKS(1000 * s_wifi_retry_count)); // backoff
                    esp_wifi_connect();
                } else {
                    ESP_LOGE(WIFI_TAG, "Max retries reached");
                    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                }
                break;
            }

            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(WIFI_TAG, "Associated with AP");
                break;

            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(WIFI_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        s_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        // Log to serial console so webapp can see the IP
        char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
        serial_console_print("%sWIFI: Connected, IP: %s\n", _ts, ip);

        // Start NTP on every (re)connect — time_manager handles dedup
        if (!esp_sntp_enabled()) {
            time_manager_start_ntp();
        }
    }
}

// ─── Time Manager Integration ────────────────────────────────────────────────
// NTP start/stop managed by time_manager.h (included at top of this file).

// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * Set WiFi credentials at runtime (before or after init).
 * If called after init, takes effect on next reconnect.
 */
static inline void wifi_hosted_set_credentials(const char *ssid, const char *password) {
    strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
    s_wifi_ssid[sizeof(s_wifi_ssid) - 1] = '\0';
    strncpy(s_wifi_pass, password, sizeof(s_wifi_pass) - 1);
    s_wifi_pass[sizeof(s_wifi_pass) - 1] = '\0';
}

/**
 * Initialize WiFi via ESP-Hosted.
 * Powers on the C6, starts ESP-Hosted SDIO transport, connects to AP.
 *
 * Degrades gracefully if the C6 doesn't have ESP-Hosted firmware:
 *   - esp_wifi_init() runs on a separate task with a 3-second timeout
 *   - If the C6 doesn't respond on SDIO, returns ESP_ERR_TIMEOUT
 *   - All partial init is cleaned up (C6 powered off, netif destroyed)
 *   - Caller prints a message and continues without WiFi
 *
 * Returns ESP_OK on success, error code on failure.
 * Call AFTER SD card is mounted on SPI (not SDMMC).
 */

// ── esp_wifi_init timeout wrapper ────────────────────────────────────────────
// esp_wifi_init() establishes the ESP-Hosted SDIO transport to the C6.
// If the C6 has no ESP-Hosted firmware, this call hangs indefinitely
// waiting for the SDIO slave to respond.  We run it on a dedicated task
// and wait on a semaphore with a timeout to detect this case.
//
// The context and semaphore are static so the task can safely write to
// them even if we time out and return — avoids use-after-free on stack.

struct wifi_init_ctx_t {
    wifi_init_config_t cfg;
    esp_err_t result;
    SemaphoreHandle_t done;
};

static struct wifi_init_ctx_t s_wifi_init_ctx;

static void wifi_init_task_fn(void *arg) {
    struct wifi_init_ctx_t *ctx = (struct wifi_init_ctx_t *)arg;
    ctx->result = esp_wifi_init(&ctx->cfg);
    // Semaphore may have been taken by the caller already (timeout path).
    // xSemaphoreGive is safe regardless — worst case it goes to max count.
    if (ctx->done) xSemaphoreGive(ctx->done);
    vTaskDelete(NULL);
}

static esp_err_t wifi_init_with_timeout(wifi_init_config_t *cfg, uint32_t timeout_ms) {
    s_wifi_init_ctx.cfg = *cfg;
    s_wifi_init_ctx.result = ESP_ERR_TIMEOUT;
    s_wifi_init_ctx.done = xSemaphoreCreateBinary();
    if (!s_wifi_init_ctx.done) return ESP_ERR_NO_MEM;

    TaskHandle_t task = NULL;
    BaseType_t ret = xTaskCreateWithCaps(wifi_init_task_fn, "wifi_probe", 4096,
                                 &s_wifi_init_ctx, 5, &task, MALLOC_CAP_SPIRAM);
    if (ret != pdPASS) {
        vSemaphoreDelete(s_wifi_init_ctx.done);
        s_wifi_init_ctx.done = NULL;
        return ESP_ERR_NO_MEM;
    }

    bool completed = (xSemaphoreTake(s_wifi_init_ctx.done,
                                     pdMS_TO_TICKS(timeout_ms)) == pdTRUE);

    if (!completed) {
        // Task is stuck in SDIO — leave it running.  SDIO Slot 1 is
        // dedicated to ESP-Hosted, so no resource conflict with SD card
        // (which uses SPI3).  The ~200B semaphore + task stack leak is
        // acceptable for a one-time error path.
        ESP_LOGE(WIFI_TAG, "esp_wifi_init timed out after %lums — "
                 "C6 not responding on SDIO (no ESP-Hosted firmware?)",
                 (unsigned long)timeout_ms);
        return ESP_ERR_TIMEOUT;
    }

    vSemaphoreDelete(s_wifi_init_ctx.done);
    s_wifi_init_ctx.done = NULL;
    return s_wifi_init_ctx.result;
}

static inline esp_err_t wifi_hosted_init(void) {
    if (s_wifi_initialized) {
        ESP_LOGW(WIFI_TAG, "Already initialized");
        return ESP_OK;
    }

    if (s_wifi_ssid[0] == '\0') {
        ESP_LOGW(WIFI_TAG, "No SSID configured — skipping WiFi init");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(WIFI_TAG, "=== Initializing WiFi (ESP-Hosted over SDIO) ===");

    // ── 1. Power on the C6 ──
    ESP_LOGI(WIFI_TAG, "Enabling ESP32-C6...");
    xl9535_c6_enable(true);
    vTaskDelay(pdMS_TO_TICKS(100));  // C6 boot time

    // ── 2. Initialize networking stack ──
    // These are idempotent — Ethernet_Init() may have already called them.
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(WIFI_TAG, "netif init failed: %s", esp_err_to_name(err));
        goto fail_c6;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(WIFI_TAG, "Event loop create failed: %s", esp_err_to_name(err));
        goto fail_c6;
    }
    s_sta_netif = esp_netif_create_default_wifi_sta();

    // ── 3. Initialize WiFi (ESP-Hosted SDIO transport to C6) ──
    // This is the critical call that communicates with the C6.  If the C6
    // doesn't have ESP-Hosted firmware, this either returns an error or
    // hangs.  The timeout wrapper handles both cases.
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = wifi_init_with_timeout(&cfg, 3000);  // 3s timeout
        if (err != ESP_OK) {
            if (err == ESP_ERR_TIMEOUT) {
                ESP_LOGE(WIFI_TAG, "C6 does not have ESP-Hosted firmware — WiFi unavailable");
                ESP_LOGE(WIFI_TAG, "Flash ESP-Hosted network_adapter to C6 to enable WiFi");
            } else {
                ESP_LOGE(WIFI_TAG, "esp_wifi_init failed: %s — WiFi unavailable",
                         esp_err_to_name(err));
            }
            goto fail_netif;
        }
    }
    ESP_LOGI(WIFI_TAG, "ESP-Hosted transport established (C6 responding)");

    // ── 4. Register event handlers ──
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(WIFI_TAG, "Failed to create event group");
        err = ESP_ERR_NO_MEM;
        goto fail_wifi;
    }

    err = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "WiFi event handler register failed: %s", esp_err_to_name(err));
        goto fail_event;
    }
    err = esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "IP event handler register failed: %s", esp_err_to_name(err));
        goto fail_event;
    }

    // ── 5. Configure and start ──
    {
        wifi_config_t wifi_config = {};
        strncpy((char *)wifi_config.sta.ssid, s_wifi_ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char *)wifi_config.sta.password, s_wifi_pass, sizeof(wifi_config.sta.password));
        wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

        err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (err != ESP_OK) {
            ESP_LOGE(WIFI_TAG, "set_mode failed: %s", esp_err_to_name(err));
            goto fail_event;
        }
        err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        if (err != ESP_OK) {
            ESP_LOGE(WIFI_TAG, "set_config failed: %s", esp_err_to_name(err));
            goto fail_event;
        }
        err = esp_wifi_start();
        if (err != ESP_OK) {
            ESP_LOGE(WIFI_TAG, "wifi_start failed: %s", esp_err_to_name(err));
            goto fail_event;
        }
    }

    s_wifi_initialized = true;
    s_wifi_paused = false;
    ESP_LOGI(WIFI_TAG, "WiFi started — connecting in background");

    // Non-blocking: connection happens asynchronously via event handler.
    // IP_EVENT → starts NTP. No 30s boot delay.
    // Caller can poll wifi_hosted_is_connected() if needed.
    return ESP_OK;

    // ── Cleanup on failure ─────────────────────────────────────────────────
    // Unwind partial initialization so the system can continue without WiFi.
    // Everything downstream (MQTT, NTP, OTA) gates on s_wifi_initialized or
    // s_wifi_has_ip, so leaving these false is sufficient.
fail_event:
    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }
fail_wifi:
    esp_wifi_deinit();
fail_netif:
    if (s_sta_netif) {
        esp_netif_destroy_default_wifi(s_sta_netif);
        s_sta_netif = NULL;
    }
fail_c6:
    xl9535_c6_enable(false);  // power off C6 — nothing to talk to
    ESP_LOGW(WIFI_TAG, "Continuing without WiFi — all other features remain functional");
    return err;
}

/**
 * Pause WiFi — graceful teardown for SD SDMMC mode swap.
 * Disconnects, stops WiFi, powers down C6.
 * Call wifi_hosted_resume() to restart.
 */
extern "C" void wifi_hosted_pause(void) {
    if (!s_wifi_initialized || s_wifi_paused) return;

    ESP_LOGI(WIFI_TAG, "Pausing WiFi...");
    s_wifi_paused = true;

    time_manager_stop_ntp();
    esp_wifi_disconnect();
    esp_wifi_stop();

    // Power down C6 to fully release SDIO bus
    xl9535_c6_enable(false);
    vTaskDelay(pdMS_TO_TICKS(50));

    s_wifi_connected = false;
    ESP_LOGI(WIFI_TAG, "WiFi paused, C6 powered down");
}

/**
 * Resume WiFi after a pause.
 * Powers on C6 and reconnects.
 */
extern "C" void wifi_hosted_resume(void) {
    if (!s_wifi_initialized || !s_wifi_paused) return;

    ESP_LOGI(WIFI_TAG, "Resuming WiFi...");

    // Power on C6
    xl9535_c6_enable(true);
    vTaskDelay(pdMS_TO_TICKS(200));  // C6 boot time

    s_wifi_paused = false;
    s_wifi_retry_count = 0;

    esp_wifi_start();  // will trigger STA_START → connect
    ESP_LOGI(WIFI_TAG, "WiFi resuming, reconnecting...");
}

/**
 * Full WiFi shutdown — deinit everything.
 * After this, wifi_hosted_init() must be called again.
 */
static inline void wifi_hosted_deinit(void) {
    if (!s_wifi_initialized) return;

    ESP_LOGI(WIFI_TAG, "Deinitializing WiFi...");
    s_wifi_paused = true; // prevent reconnect attempts

    time_manager_stop_ntp();

    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_sta_netif) {
        esp_netif_destroy_default_wifi(s_sta_netif);
        s_sta_netif = NULL;
    }

    xl9535_c6_enable(false);

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_wifi_initialized = false;
    s_wifi_connected = false;
    s_wifi_paused = false;
    ESP_LOGI(WIFI_TAG, "WiFi deinitialized");
}

/**
 * Check if WiFi is currently connected.
 */
extern "C" bool wifi_hosted_is_active(void) {
    return s_wifi_initialized && !s_wifi_paused;
}

static inline bool wifi_hosted_is_connected(void) {
    return s_wifi_connected;
}

/**
 * Get the current IP address as a string.
 * Returns "0.0.0.0" if not connected.
 */
static inline const char *wifi_hosted_get_ip(void) {
    static char ip_str[16] = "0.0.0.0";
    if (s_sta_netif && s_wifi_connected) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        }
    }
    return ip_str;
}
