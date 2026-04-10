// mqtt_feeder.h — MQTT publisher for ADS-B aircraft data
//
// Connects to an MQTT broker over TLS and publishes:
//   adsb/{device_id}/aircraft  — batched aircraft positions (JSON, QoS 0)
//   adsb/{device_id}/status    — receiver status heartbeat (JSON, QoS 1)
//   adsb/{device_id}/meta      — device metadata on connect (JSON, QoS 1, retained)
//
// Usage:
//   Call mqtt_feeder_init() once after WiFi is connected.
//   Call mqtt_feeder_update_aircraft() from the ADS-B decode loop.
//   Call mqtt_feeder_update_status() periodically (e.g. from heartbeat timer).
//   The feeder task handles batching, serialization, and publishing.
//
// Memory: task stack and batch buffer are PSRAM-allocated.
//         MQTT library TLS buffers (~8KB) use internal RAM (DMA requirement).
//
// Dependencies: esp_mqtt, cJSON (both included in ESP-IDF)

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ─── Aircraft snapshot (queued from decode loop) ─────────────────────────────

typedef struct {
    char     icao[9];        // hex ICAO address (6 chars + padding for compiler)
    char     callsign[9];    // flight callsign or empty
    double   lat, lon;       // position (0,0 = no position)
    int32_t  alt_ft;         // altitude in feet
    int16_t  speed_kt;       // ground speed
    int16_t  heading_deg;    // track heading
    int16_t  vert_rate_fpm;  // vertical rate
    float    dist_nm;        // distance from receiver
    int16_t  bearing_deg;    // bearing from receiver
    int64_t  ts_us;          // device timestamp (microseconds)
    bool     has_pos;        // has valid position
} mqtt_aircraft_t;

// ─── Receiver status snapshot ────────────────────────────────────────────────

typedef struct {
    double   lat, lon;       // receiver position
    uint8_t  sats;
    float    hdop;
    float    gain_db;
    uint8_t  gain_phase;     // 1=converging, 2=steady
    float    error_rate;
    float    pos_rate;
    float    max_range_nm;
    uint16_t ac_count;
    uint32_t msg_count;
    float    msg_rate;
    uint32_t mem_free;
    uint32_t mem_psram;
    uint32_t uptime_s;
    char     firmware_ver[32];
    char     dongle_model[64];   // RTL-SDR dongle description (e.g. "RTLSDRBlog Blog V4 (R828D)")
} mqtt_status_t;

// ─── Public API ──────────────────────────────────────────────────────────────

// Initialize and start the MQTT feeder. Call once after WiFi first gets an IP.
// Returns immediately (no-op) if MQTT is disabled in settings or not configured.
// All allocations use PSRAM where possible.
void mqtt_feeder_init(void);

// Pause MQTT — stop the client and cancel pending retries.
// Call when WiFi disconnects. Safe to call if not initialized.
void mqtt_feeder_pause(void);

// Resume MQTT — restart the client after WiFi reconnects.
// Call from WiFi got-IP handler. Safe to call if not initialized.
void mqtt_feeder_resume(void);

// Queue an aircraft update from the decode loop.
// Returns immediately if not connected — zero overhead on the decode path.
// Mutex wait is capped at 5ms; drops the update if lock isn't available.
void mqtt_feeder_update_aircraft(const mqtt_aircraft_t *ac);

// Update the receiver status snapshot. Called from heartbeat timer (~30s).
void mqtt_feeder_update_status(const mqtt_status_t *st);

// Shutdown cleanly. Stops task, disconnects, frees resources.
void mqtt_feeder_stop(void);

// Check if the MQTT feeder is currently connected to the broker.
bool mqtt_feeder_is_connected(void);

// Get the device ID string (e.g. "p4-0ad603404ad3"). Generated from MAC address
// on first call. Always available, even without WiFi/MQTT.
const char *mqtt_feeder_device_id(void);

#ifdef __cplusplus
}
#endif
