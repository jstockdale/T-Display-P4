// device_settings.h — Persistent device settings backed by NVS flash
// Versioned blob with forward-compatible migration.
//
// RULES FOR FUTURE CHANGES:
//   1. Bump SETTINGS_VERSION
//   2. Add new fields ONLY at the end (before _pad)
//   3. Reduce _pad size by the bytes you added
//   4. The memcpy migration in settings_load() handles it automatically:
//      old blob is smaller, new fields stay at their defaults.
//   5. If you must reorder fields (DON'T), add an explicit migration case.
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tz_lookup.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SETTINGS_NVS_NAMESPACE "device_settings"
#define SETTINGS_VERSION       13   // bump when struct changes

// ─── Settings structure ──────────────────────────────────────────────────────

typedef struct {
    // ── Display ──
    uint8_t  brightness;          // 0-100%, default 80
    uint16_t screen_timeout_s;    // 0 = never, default 120 (2 min)
    bool     double_tap_wake;     // IMU double-tap toggles display, default true
    bool     auto_rotation;       // auto-rotate based on orientation, default false

    // ── Time & Date ──
    int8_t   tz_offset_h;        // timezone hours offset from UTC, -12 to +14
    int8_t   tz_offset_m;        // timezone minutes offset (0 or 30)
    bool     dst_enabled;         // daylight saving time active, default false
    bool     time_24h;            // 24h format, default true
    bool     show_seconds;        // show seconds on clock, default false
    bool     tz_auto;             // true=GPS auto timezone, false=manual, default true

    // ── ADS-B ──
    bool     adsb_enabled;        // enable RTL-SDR receiver, default true
    bool     adsb_sd_logging;     // log to SD card, default true
    bool     adsb_manual_pos;     // use manual position instead of GPS
    double   adsb_manual_lat;     // manual receiver latitude
    double   adsb_manual_lon;     // manual receiver longitude
    bool     adsb_bias_tee;       // bias-T power on antenna port (4.5V DC), default false

    // ── Meshtastic ──
    bool     meshy_enabled;       // enable LoRa radio, default true
    bool     meshy_sd_logging;    // log to SD card, default true
    uint8_t  meshy_region;        // 0=US..14=SG
    uint8_t  meshy_preset;        // 0=LongFast..5=ShortSlow
    uint8_t  meshy_tx_power;      // dBm, default 30
    char     meshy_node_name[16]; // short name, default "" (auto from MAC)
    uint8_t  meshy_freq_slot;     // 0=auto (hash), 1+=explicit slot, default 0
    uint8_t  meshy_role;          // 0=Client, 1=ClientMute, 2=RouterLate
    bool     meshy_ok_to_mqtt;    // set ok_to_mqtt bit on outgoing packets, default true
    uint16_t meshy_nodeinfo_period_m; // NODEINFO broadcast period in minutes (15-1440), default 30
    uint8_t  meshy_hop_limit;    // TX hop limit (1-7), default 5

    // ── GPS ──
    bool     gps_enabled;         // enable L76K GPS, default true

    // ── Audio & Haptics ──
    uint8_t  volume;              // 0-100%, default 50
    bool     haptic_enabled;      // vibration feedback, default true

    // ── Scope Display ──
    uint8_t  scope_fps_cap;       // 0=auto, 5/10/15/30, default 0 (auto)
    uint8_t  scope_max_aircraft;  // 0=unlimited (up to 128), default 0

    // ── Console ──
    bool     heartbeat_enabled;    // serial console heartbeat, default true
    uint8_t  heartbeat_period_s;   // heartbeat period in seconds (5-255), default 30

    // ── WiFi ──
    bool     wifi_enabled;         // enable WiFi via ESP-Hosted (C6), default false
    char     wifi_ssid[33];        // SSID, null-terminated, default ""
    char     wifi_pass[65];        // password, null-terminated, default ""
    char     ntp_server[64];       // NTP server, default "pool.ntp.org"
    uint16_t ntp_poll_s;           // NTP poll interval in seconds (300-3600), default 300

    // ── ADS-B Gain ──
    uint16_t adsb_gain_tenths;    // gain in tenths of dB (e.g. 496 = 49.6 dB), default 496
    uint8_t  adsb_gain_mode;      // 0=auto (adaptive), 1=manual (fixed), default 0

    // ── GPS Integrity ──
    uint8_t  gps_integrity_mode;  // 0=basic, 1=strict, default 0 (see GPS_INTEGRITY_*)

    // ── MQTT Feeder ──
    bool     mqtt_enabled;         // enable MQTT publishing, default true
    char     mqtt_server[64];      // broker hostname, default "mqtt.offx1.com"
    char     mqtt_user[33];        // broker username, default "adsb-scope-alpha"
    char     mqtt_pass[41];        // broker password, default (see below)
    char     mqtt_device_id[33];   // unique device identifier, default "" (auto from MAC)
    uint16_t mqtt_port;            // broker port, default 8883 (MQTTS)

    // ── Gain Target Band ──
    uint8_t  gain_err_low;         // lower bound of target error rate (percent), default 40
    uint8_t  gain_err_high;        // upper bound of target error rate (percent), default 50

    // ── ADD NEW FIELDS HERE — consume _pad bytes, bump SETTINGS_VERSION ──

    // ── Reserved for future fields ──
    uint8_t  _pad[610];
} device_settings_t;

// Struct must be exactly 1024 bytes — adjust _pad if this fires
// (alignment may differ on RISC-V vs x86; the assert catches it)
_Static_assert(sizeof(device_settings_t) == 1024, "device_settings_t must be 1024 bytes — adjust _pad");

// ─── Default values ──────────────────────────────────────────────────────────

static inline void settings_apply_defaults(device_settings_t *s) {
    memset(s, 0, sizeof(*s));
    s->brightness       = 80;
    s->screen_timeout_s = 120;
    s->double_tap_wake  = true;
    s->auto_rotation    = false;
    s->tz_offset_h      = -8;    // PST
    s->tz_offset_m      = 0;
    s->dst_enabled      = true;  // PDT
    s->time_24h         = true;
    s->show_seconds     = false;
    s->tz_auto          = true;
    s->adsb_enabled     = true;
    s->adsb_sd_logging  = true;
    s->adsb_manual_pos  = false;
    s->adsb_manual_lat  = 0.0;
    s->adsb_manual_lon  = 0.0;
    s->adsb_bias_tee    = false;
    s->meshy_enabled    = true;
    s->meshy_sd_logging = true;
    s->meshy_region     = 0;     // US
    s->meshy_preset     = 2;     // MediumFast
    s->meshy_tx_power   = 30;
    s->meshy_node_name[0] = '\0';
    s->meshy_freq_slot  = 0;     // auto (hash-based)
    s->meshy_role       = 0;     // Client
    s->meshy_ok_to_mqtt = true;
    s->meshy_nodeinfo_period_m = 30;
    s->meshy_hop_limit  = 5;
    s->gps_enabled      = true;
    s->volume           = 50;
    s->haptic_enabled   = true;
    s->scope_fps_cap    = 0;     // auto
    s->scope_max_aircraft = 0;   // unlimited
    s->heartbeat_enabled = true;
    s->heartbeat_period_s = 30;
    s->wifi_enabled     = false;   // offline-first: WiFi off by default
    s->wifi_ssid[0]     = '\0';
    s->wifi_pass[0]     = '\0';
    strncpy(s->ntp_server, "pool.ntp.org", sizeof(s->ntp_server));
    s->ntp_poll_s       = 300;     // 5 minutes
    s->adsb_gain_tenths = 496;     // 49.6 dB (R820T max — adaptive will adjust)
    s->adsb_gain_mode   = 0;       // auto (adaptive)
    s->gps_integrity_mode = 0;     // basic (garbled NMEA guard only)
    // MQTT feeder — enabled by default with alpha credentials
    s->mqtt_enabled     = true;
    strncpy(s->mqtt_server, "mqtt.offx1.com", sizeof(s->mqtt_server));
    strncpy(s->mqtt_user, "adsb-scope-alpha", sizeof(s->mqtt_user));
    strncpy(s->mqtt_pass, "d744212ba48991327c3846c51a864828581af730", sizeof(s->mqtt_pass));
    s->mqtt_device_id[0]= '\0';   // empty = auto from MAC at boot
    s->mqtt_port        = 8883;   // MQTTS default
    // Gain target band — algorithm seeks error rate within this band
    s->gain_err_low     = 40;     // 40% — below this, step up (deaf to distant aircraft)
    s->gain_err_high    = 50;     // 50% — above this, step down (saturation encroaching)
}

// Legacy wrapper for any code that calls settings_defaults()
static inline device_settings_t settings_defaults(void) {
    device_settings_t s;
    settings_apply_defaults(&s);
    return s;
}

// ─── Validation — clamp all values to safe ranges ────────────────────────────

static inline void settings_validate(device_settings_t *s) {
    if (s->brightness < 5 || s->brightness > 100) s->brightness = 80;
    if (s->screen_timeout_s > 600) s->screen_timeout_s = 120;
    if (s->tz_offset_h < -12 || s->tz_offset_h > 14) s->tz_offset_h = -8;
    if (s->meshy_region > 14) s->meshy_region = 0;
    if (s->meshy_preset > 5) s->meshy_preset = 2;
    if (s->meshy_freq_slot > 200) s->meshy_freq_slot = 0;  // 0=auto, 1+=slot
    if (s->meshy_role > 2) s->meshy_role = 0;
    if (s->meshy_nodeinfo_period_m < 15 || s->meshy_nodeinfo_period_m > 1440)
        s->meshy_nodeinfo_period_m = 30;
    if (s->meshy_tx_power > 30) s->meshy_tx_power = 30;
    if (s->meshy_hop_limit < 1 || s->meshy_hop_limit > 7) s->meshy_hop_limit = 5;
    if (s->volume > 100) s->volume = 50;
    if (s->scope_fps_cap > 30) s->scope_fps_cap = 0;
    if (s->scope_max_aircraft > 128) s->scope_max_aircraft = 0; // MAX_SCOPE_AIRCRAFT
    if (s->heartbeat_period_s < 5) s->heartbeat_period_s = 30;
    // WiFi: ensure null-termination and valid poll interval
    s->wifi_ssid[32] = '\0';
    s->wifi_pass[64] = '\0';
    s->ntp_server[63] = '\0';
    if (s->ntp_server[0] == '\0') strncpy(s->ntp_server, "pool.ntp.org", sizeof(s->ntp_server));
    if (s->ntp_poll_s < 300 || s->ntp_poll_s > 3600) s->ntp_poll_s = 300;
    // ADS-B gain
    if (s->adsb_gain_mode > 1) s->adsb_gain_mode = 0;
    if (s->adsb_gain_tenths > 496) s->adsb_gain_tenths = 496;
    // GPS integrity
    if (s->gps_integrity_mode > 1) s->gps_integrity_mode = 0;
    // MQTT feeder
    s->mqtt_server[63] = '\0';
    s->mqtt_user[32] = '\0';
    s->mqtt_pass[40] = '\0';
    s->mqtt_device_id[32] = '\0';
    if (s->mqtt_port == 0) s->mqtt_port = 8883;
    // Gain target band: safe operational range — keeps derived thresholds sane
    // very_high (high+5) must stay below SEVERE (67%), very_low (low-20) must stay above 0%
    if (s->gain_err_low < 25 || s->gain_err_low > 55) s->gain_err_low = 40;
    if (s->gain_err_high < 35 || s->gain_err_high > 60) s->gain_err_high = 50;
    if (s->gain_err_high < s->gain_err_low + 10) s->gain_err_high = s->gain_err_low + 10;
    if (s->gain_err_high > s->gain_err_low + 20) s->gain_err_high = s->gain_err_low + 20;
    if (s->gain_err_high > 60) { s->gain_err_high = 60; if (s->gain_err_low > 50) s->gain_err_low = 50; }
}

// ─── Global settings instance (defined in main.cpp) ──────────────────────────

extern device_settings_t g_settings;

// Screen timeout state (defined in main.cpp)
extern volatile uint32_t g_last_touch_ms;
extern volatile bool g_screen_blanked;

// ─── Deferred NVS persistence ─────────────────────────────────────────────────

// Defined in main.cpp alongside g_settings
extern volatile bool g_settings_save_pending;
extern volatile bool g_settings_reset_pending;

// ─── Load with version migration ─────────────────────────────────────────────

static inline void settings_load(void) {
    settings_apply_defaults(&g_settings);

    nvs_handle_t nvs;
    if (nvs_open(SETTINGS_NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGW("SETTINGS", "No saved settings found, using defaults");
        return;
    }

    // Read stored version (separate key, not in blob)
    uint8_t stored_version = 0;
    nvs_get_u8(nvs, "ver", &stored_version);

    if (stored_version == 0) {
        // Pre-versioned blob — cannot migrate safely, use defaults
        ESP_LOGW("SETTINGS", "Unversioned settings (v0) — using defaults");
        nvs_close(nvs);
        return;
    }

    // Read blob into temp buffer (static — main task stack is only ~3.5KB)
    static uint8_t raw[1024];
    size_t len = sizeof(raw);
    if (nvs_get_blob(nvs, "cfg", raw, &len) != ESP_OK) {
        ESP_LOGW("SETTINGS", "Failed to read settings blob, using defaults");
        nvs_close(nvs);
        return;
    }
    nvs_close(nvs);

    if (stored_version > SETTINGS_VERSION) {
        // Downgrade — blob is from newer firmware, don't trust it
        ESP_LOGW("SETTINGS", "Settings v%d > firmware v%d — using defaults",
                 stored_version, SETTINGS_VERSION);
        return;
    }

    // Migration: g_settings starts as defaults. Copy the old blob over it.
    // Since new fields are always appended at the end (before _pad):
    //   - Old fields at the same offsets get overwritten with saved values
    //   - New fields beyond the old blob size keep their defaults
    //   - _pad is always zero
    size_t copy = (len < sizeof(g_settings)) ? len : sizeof(g_settings);
    memcpy(&g_settings, raw, copy);

    if (stored_version < SETTINGS_VERSION) {
        ESP_LOGI("SETTINGS", "Migrated v%d -> v%d (%zu bytes)", stored_version, SETTINGS_VERSION, len);
        // Future: add explicit field fixups here if a migration is non-trivial
        // switch (stored_version) {
        //     case 2: /* v2->v3: new_field was added, default is fine */ break;
        // }
    } else {
        ESP_LOGI("SETTINGS", "Loaded v%d settings (%zu bytes)", stored_version, len);
    }

    settings_validate(&g_settings);
}

// Called from LVGL callbacks (PSRAM stack) — just sets a flag
static inline void settings_save(void) {
    g_settings_save_pending = true;
}

// ── Internal: NVS write task (runs on internal RAM stack, self-deletes) ──────

static void _settings_nvs_write_task(void *arg) {
    bool is_reset = (bool)(uintptr_t)arg;

    if (is_reset) {
        nvs_handle_t nvs;
        if (nvs_open(SETTINGS_NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_erase_all(nvs);
            nvs_set_u8(nvs, "ver", SETTINGS_VERSION);
            nvs_set_blob(nvs, "cfg", &g_settings, sizeof(g_settings));
            nvs_commit(nvs);
            nvs_close(nvs);
        }
        ESP_LOGI("SETTINGS", "Factory reset complete — defaults saved (v%d)", SETTINGS_VERSION);
    } else {
        nvs_handle_t nvs;
        if (nvs_open(SETTINGS_NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) {
            ESP_LOGE("SETTINGS", "Failed to open NVS for writing");
        } else {
            nvs_set_u8(nvs, "ver", SETTINGS_VERSION);
            if (nvs_set_blob(nvs, "cfg", &g_settings, sizeof(g_settings)) != ESP_OK) {
                ESP_LOGE("SETTINGS", "Failed to write settings blob");
            } else {
                nvs_commit(nvs);
                ESP_LOGI("SETTINGS", "Settings saved (v%d)", SETTINGS_VERSION);
            }
            nvs_close(nvs);
        }
    }

    vTaskDelete(NULL);
}

// Polled from any task (safe from PSRAM stacks).
// Spawns a short-lived internal-RAM task for the actual NVS write,
// because SPI flash operations disable the cache (PSRAM inaccessible).
static inline void settings_save_if_pending(void) {
    if (g_settings_reset_pending) {
        g_settings_reset_pending = false;
        g_settings_save_pending = false;
        // xTaskCreate allocates stack from internal RAM by default
        xTaskCreate(_settings_nvs_write_task, "nvs_wr", 3072,
                    (void *)(uintptr_t)true, 5, NULL);
        return;
    }

    if (!g_settings_save_pending) return;
    g_settings_save_pending = false;

    xTaskCreate(_settings_nvs_write_task, "nvs_wr", 3072,
                (void *)(uintptr_t)false, 5, NULL);
}

// Called from LVGL callbacks (PSRAM stack) — deferred
static inline void settings_reset(void) {
    settings_apply_defaults(&g_settings);
    g_settings_reset_pending = true;
}

// ─── Helper: total UTC offset in minutes (including DST) ─────────────────────

static inline int settings_utc_offset_minutes(void) {
    int offset = g_settings.tz_offset_h * 60 + g_settings.tz_offset_m;
    if (g_settings.dst_enabled) offset += 60;
    return offset;
}

// ─── String helpers for UI ──────────────────────────────────────────────────

static inline const char *settings_region_name(uint8_t region) {
    static const char *names[] = {
        "US", "EU_868", "EU_433", "CN", "JP", "ANZ", "KR", "TW",
        "RU", "IN", "NZ", "TH", "UA", "MY", "SG"
    };
    return (region < sizeof(names)/sizeof(names[0])) ? names[region] : "?";
}

static inline const char *settings_preset_name(uint8_t preset) {
    static const char *names[] = {
        "LongFast", "LongSlow", "MediumFast", "MediumSlow", "ShortFast", "ShortSlow"
    };
    return (preset < sizeof(names)/sizeof(names[0])) ? names[preset] : "?";
}

static inline const char *settings_role_name(uint8_t role) {
    static const char *names[] = { "Client", "Client Mute", "Router Late" };
    return (role < sizeof(names)/sizeof(names[0])) ? names[role] : "?";
}

static inline const char *settings_tz_string(void) {
    static char buf[24];
    if (g_settings.tz_auto) {
        snprintf(buf, sizeof(buf), "Auto (GPS)");
    } else {
        int h = g_settings.tz_offset_h;
        int m = g_settings.tz_offset_m;
        snprintf(buf, sizeof(buf), "UTC%+d:%02d%s", h, m < 0 ? -m : m,
                 g_settings.dst_enabled ? " DST" : "");
    }
    return buf;
}

// Format an offset in minutes as "UTC±H:MM" or "UTC±H:MM DST"
static inline const char *settings_format_offset(int16_t offset_min, bool dst_active) {
    static char buf[24];
    int h = offset_min / 60;
    int m = abs(offset_min) % 60;
    snprintf(buf, sizeof(buf), "UTC%+d:%02d%s", h, m, dst_active ? " DST" : "");
    return buf;
}

/**
 * Apply timezone settings to the tz_lookup runtime module.
 * Call after any change to tz_auto / tz_offset_h / tz_offset_m / dst_enabled.
 */
static inline void settings_apply_timezone(void) {
    if (g_settings.tz_auto) {
        tz_set_auto();
    } else {
        int16_t total = (int16_t)(g_settings.tz_offset_h * 60 + g_settings.tz_offset_m);
        if (g_settings.dst_enabled) total += 60;
        tz_set_manual_offset(total);
    }
}

#ifdef __cplusplus
}
#endif
