/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class_driver.h"
#include "serial_console.h"
#include "sd_config.h"
#include "device_settings.h"
#include "aircraft_db.h"
#include "mqtt_feeder.h"
#include "rtlsdr_i2c.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

// GPS fix epoch — set in GPS task on first quality fix.
// Trusted time check — true when GPS, NTP, or PPS has synced (not just RTC boot seed).
// Used to gate CSV timestamp format and SD log renames.
extern bool time_manager_is_trusted_fn(void);
// SD I/O mutex — defined in main.cpp, serializes file I/O across tasks
extern bool sd_io_take(uint32_t timeout_ms);
extern void sd_io_give(void);

#define RTLSDR_BUF_LEN        (16384 + 512)
#define CLIENT_NUM_EVENT_MSG  5
#define CPR_MAX_AGE_US        10000000LL   // 10 seconds
#define CPR_CACHE_SIZE        256
#define CPR_JUMP_FLOOR_NM     10.0         // minimum threshold (handles GPS jitter, CPR quantization)
#define CPR_JUMP_RATE_NM_S    1.0          // ~3600 kt — faster than any transponder-equipped aircraft
#define CPR_SUSPECT_GIVE_UP   5            // consecutive suspect events → reset has_position
#define CPR_CONSENSUS_COUNT   3            // agreeing globals needed to override bad good_pos
#define CPR_CONSENSUS_NM      10.0         // max spread between agreeing candidates
#define AIRCRAFT_TABLE_SIZE   256
#define AIRCRAFT_MAX_AGE_US   60000000LL   // 60 seconds

#define ADSB_READER_TASK_STACK  (8 * 1024)
#define ADSB_READER_TASK_PRIO   3

// ============================================================
// ADSB reader task signaling
// ============================================================

static TaskHandle_t s_adsb_reader_task_hdl = NULL;
static volatile bool s_adsb_reader_stop = false;

// When true, class_driver_task must NOT call usb_host_client_handle_events —
// the reader task becomes the sole USB event consumer to avoid concurrent
// event pumping on the same client handle (which corrupts the heap).
static volatile bool s_reader_owns_events = false;

// Passed from event callback to the reader task
static volatile uint8_t s_new_dev_addr = 0;

// Pre-allocated demodulation magnitude buffer (allocated once in reader task)
static uint16_t *s_mag_buf = NULL;

// ADS-B statistics — updated by on_msg, read by adsb_get_stats
static volatile uint32_t s_total_messages = 0;
static volatile uint32_t s_msg_count_window = 0;  // messages in current 1s window
static volatile float    s_msg_rate = 0.0f;       // messages per second
static volatile int64_t  s_msg_window_start = 0;  // start of current rate window
static volatile bool     s_rtlsdr_connected = false;
static volatile bool     s_rtlsdr_error = false;  // device seen but transfer buffer failed

// ============================================================
// Adaptive tuner gain — throughput-optimized two-phase
// ============================================================
// R820T valid gain steps in tenths of dB (from datasheet)
static const uint16_t R820T_GAINS[] = {
    0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
    166, 197, 207, 229, 254, 280, 297, 328, 338,
    364, 372, 386, 402, 421, 434, 439, 445, 480, 496
};
#define R820T_GAIN_COUNT (sizeof(R820T_GAINS) / sizeof(R820T_GAINS[0]))
#define GAIN_FLOOR_IDX      11     // 19.7 dB — never go below this
#define GAIN_MIN_SAMPLES    10     // need enough decoded messages to evaluate

// Phase 1 (fast convergence): eval every 10s or 200 msgs
#define GAIN_P1_PERIOD_US   10000000LL
#define GAIN_P1_MSG_COUNT   200
#define GAIN_P1_STABLE_WIN  3      // consecutive stable windows → converged
#define GAIN_WARMUP_US      10000000LL  // 10s decoder warmup before first eval

// Phase 2 (steady-state): eval every 60s
#define GAIN_P2_PERIOD_US   60000000LL
#define GAIN_P2_EMA_ALPHA   0.3f   // EMA smoothing factor
#define GAIN_P2_COOLDOWN    2      // skip 2 windows after gain change (120s settle)
#define GAIN_P2_THROUGHPUT_DROP 0.15f // >15% pos_rate drop = bad step (beyond traffic noise)

// Thresholds — the target band is user-configurable via settings.
// Derived thresholds maintain fixed offsets from the band edges.
// SEVERE is absolute (ADC clipping), not relative to the band.
#define GAIN_ERR_SEVERE     0.67f  // phase 2 emergency — fall back to phase 1

// Signal quality gates — prevent gain reductions when signals are already weak.
// Based on preamble signal metric (avg of 4 preamble peaks per CRC-ok message):
//   Clipping regime: sig > 10000 (e.g., 13411 at 49.6 dB)
//   Good range:      sig 5000–20000 (aircraft in range, ADC not saturated)
//   Weak range:      sig < 5000 (distant/departing aircraft, barely decoding)
// High error + weak signals = noise or no traffic, NOT ADC saturation.
#define GAIN_MIN_SIG_FOR_CLIPPING  8000  // P2 emergency: must see strong signals to claim clipping
#define GAIN_MIN_SIG_FOR_DOWN      5000  // P1/P2: don't reduce gain if signals weaker than this
#define GAIN_MIN_OK_FOR_STATS      3     // need ≥3 CRC-ok messages for reliable signal metrics
static inline float gain_err_high(void) { extern device_settings_t g_settings; return g_settings.gain_err_high / 100.0f; }
static inline float gain_err_low(void)  { extern device_settings_t g_settings; return g_settings.gain_err_low / 100.0f; }
static inline float gain_err_very_high(void) { return gain_err_high() + 0.05f; }  // clear soft floor
static inline float gain_err_very_low(void)  { return gain_err_low() - 0.20f; }   // clear soft ceiling

// Soft ceiling (can't step above)
#define GAIN_CEILING_CLEAR_WIN 5   // 5 consecutive low-error windows to clear ceiling
#define GAIN_CEILING_MAX_WIN   20  // hard time cap: 20 windows (20 min) then auto-clear
#define GAIN_CEILING_STRIKES   2   // require 2 failed probes before setting ceiling

// Soft floor (can't step below)
#define GAIN_FLOOR_CLEAR_WIN   5   // 5 consecutive high-error windows to clear floor
#define GAIN_FLOOR_MAX_WIN     20  // hard time cap: 20 windows (20 min) then auto-clear
#define GAIN_FLOOR_STRIKES     2   // require 2 failed probes before setting floor

// ── Core state ──
static volatile int      s_gain_idx = R820T_GAIN_COUNT - 1;
static volatile bool     s_gain_pending = false;  // deferred gain apply — set by external callers
static volatile bool     s_gain_auto = true;
static volatile int64_t  s_gain_last_eval = 0;

// ── Phase state ──
static volatile int      s_gain_phase = 1;     // 1=fast, 2=steady
static volatile int      s_gain_stable_count = 0; // consecutive windows without gain change
static volatile float    s_gain_p1_prev_pos_rate = 0.0f; // previous window pos_rate for phase 1 comparison
static volatile uint32_t s_gain_p1_prev_ok = 0;          // previous window ok (good CRC) count
static volatile bool     s_gain_p1_last_was_up = false;   // true if last phase 1 action was a step-up
static volatile int      s_gain_p1_up_suppress = 0;       // >0 = suppress step-ups (failed step-up cooldown)
static volatile bool     s_gain_warmup = true;             // true until first eval completes

// ── Per-window position update counter (incremented in on_msg) ──
static volatile uint32_t s_gain_win_pos = 0;

// ── Phase 2: EMA-smoothed metrics ──
static volatile float    s_gain_err_ema = 0.0f;     // error rate EMA
static volatile float    s_gain_pos_rate_ema = 0.0f; // position updates/sec EMA (primary metric)
static volatile float    s_gain_error_rate = 0.0f;   // exposed to stats API

// ── Signal quality metrics (last window, exposed to stats API) ──
static volatile uint32_t s_gain_avg_signal = 0;      // avg preamble signal level (CRC-ok msgs)
static volatile uint32_t s_gain_avg_delta = 0;        // avg bit delta / SNR proxy (CRC-ok msgs)

// ── Phase 2: throughput comparison ──
static volatile float    s_gain_pre_step_pos_rate = 0.0f; // snapshot before gain change
static volatile uint32_t s_gain_pre_step_signal = 0;      // avg_signal snapshot before gain change
static volatile int      s_gain_cooldown = 0;
static volatile int      s_gain_step_dir = 0;  // +1=stepped up, -1=stepped down, 0=none
#define GAIN_P2_SIG_CHANGE_THRESH 0.40f // >40% signal change = traffic shifted, not gain damage

// ── Soft ceiling ──
static volatile int      s_gain_ceiling = R820T_GAIN_COUNT; // one past max = no ceiling
static volatile int      s_gain_low_err_count = 0;
static volatile int      s_gain_ceiling_strikes = 0;  // failed probes since last ceiling clear
static volatile int      s_gain_ceiling_age = 0;       // windows since ceiling was set

// ── Soft floor ──
static volatile int      s_gain_floor = -1;            // -1 = no floor (GAIN_FLOOR_IDX is hard limit)
static volatile int      s_gain_high_err_count = 0;
static volatile int      s_gain_floor_strikes = 0;     // failed probes since last floor clear
static volatile int      s_gain_floor_age = 0;          // windows since floor was set

// ── Historical throughput profile (EMA of pos_rate per gain step) ──
#define GAIN_PROFILE_ALPHA    0.1f  // slow EMA — profile reflects hours, not minutes
#define GAIN_PROFILE_MIN_WIN  3     // need at least 3 windows before trusting a profile entry
#define GAIN_PROFILE_MIN_DELTA 0.25f // profile must be at least 0.25/s better (prevents noise probes)
static float    s_gain_profile[R820T_GAIN_COUNT];      // EMA pos_rate at each level
static uint16_t s_gain_profile_samples[R820T_GAIN_COUNT]; // windows observed at each level

// ============================================================
// CPR cache with timestamps — linear-probed hash table
// ============================================================

typedef struct {
    uint32_t icao;
    int even_valid, odd_valid;
    int even_lat, even_lon;
    int odd_lat, odd_lon;
    int64_t even_ts, odd_ts;
    double lat, lon;
    int occupied;
} cpr_cache_t;

static cpr_cache_t cpr_cache[CPR_CACHE_SIZE];

static cpr_cache_t *cpr_get(uint32_t icao) {
    uint32_t idx = icao % CPR_CACHE_SIZE;

    // Linear probe: find existing entry or first free slot
    for (uint32_t probe = 0; probe < CPR_CACHE_SIZE; probe++) {
        uint32_t slot = (idx + probe) % CPR_CACHE_SIZE;

        if (cpr_cache[slot].occupied && cpr_cache[slot].icao == icao)
            return &cpr_cache[slot];

        if (!cpr_cache[slot].occupied) {
            memset(&cpr_cache[slot], 0, sizeof(cpr_cache_t));
            cpr_cache[slot].icao = icao;
            cpr_cache[slot].occupied = 1;
            return &cpr_cache[slot];
        }
    }

    // Table full — evict the slot with the oldest timestamp
    uint32_t oldest_slot = idx;
    int64_t oldest_ts = INT64_MAX;
    for (uint32_t i = 0; i < CPR_CACHE_SIZE; i++) {
        int64_t ts = cpr_cache[i].even_ts > cpr_cache[i].odd_ts
                   ? cpr_cache[i].even_ts : cpr_cache[i].odd_ts;
        if (ts < oldest_ts) {
            oldest_ts = ts;
            oldest_slot = i;
        }
    }
    memset(&cpr_cache[oldest_slot], 0, sizeof(cpr_cache_t));
    cpr_cache[oldest_slot].icao = icao;
    cpr_cache[oldest_slot].occupied = 1;
    return &cpr_cache[oldest_slot];
}

static int cpr_decode(cpr_cache_t *c) {
    #define NL(lat) ((lat) == 0 ? 59 : (int)(2*M_PI / acos(1 - (1-cos(M_PI/30.0)) / pow(cos(M_PI/180.0*(lat)),2))))

    // Reject stale pairs
    int64_t age = c->even_ts - c->odd_ts;
    if (age < 0) age = -age;
    if (age > CPR_MAX_AGE_US) return 0;

    double lat_even = c->even_lat / 131072.0;
    double lat_odd  = c->odd_lat  / 131072.0;

    int j = (int)floor(59 * lat_even - 60 * lat_odd + 0.5);

    double rlat_even = (360.0 / 60) * (((j % 60) + 60) % 60 + lat_even);
    double rlat_odd  = (360.0 / 59) * (((j % 59) + 59) % 59 + lat_odd);
    if (rlat_even >= 270) rlat_even -= 360;
    if (rlat_odd  >= 270) rlat_odd  -= 360;

    if (NL(rlat_even) != NL(rlat_odd)) return 0;

    // Use the most recently received frame
    double rlat = (c->even_ts >= c->odd_ts) ? rlat_even : rlat_odd;
    int nl = NL(rlat);

    double lon_even = c->even_lon / 131072.0;
    double lon_odd  = c->odd_lon  / 131072.0;

    int ni_even = nl > 1 ? nl : 1;
    int ni_odd  = (nl - 1) > 1 ? (nl - 1) : 1;

    int m;
    double rlon;
    if (c->even_ts >= c->odd_ts) {
        m    = (int)floor(lon_even * (nl - 1) - lon_odd * nl + 0.5);
        rlon = (360.0 / ni_even) * (((m % ni_even) + ni_even) % ni_even + lon_even);
    } else {
        m    = (int)floor(lon_even * (nl - 1) - lon_odd * nl + 0.5);
        rlon = (360.0 / ni_odd) * (((m % ni_odd) + ni_odd) % ni_odd + lon_odd);
    }
    if (rlon >= 180) rlon -= 360;

    c->lat = rlat;
    c->lon = rlon;
    return 1;
}

// Local CPR decode: decode a single CPR frame against a known reference position.
// Works when we already have a valid position from a previous global decode.
// The aircraft must be within ~180 nm of the reference — always true for recent positions.
static double fmod_pos(double a, double b) {
    double r = fmod(a, b);
    return r < 0 ? r + b : r;
}

static int cpr_decode_local(double ref_lat, double ref_lon, int raw_lat, int raw_lon,
                            int fflag, double *out_lat, double *out_lon) {
    double d_lat = fflag ? (360.0 / 59.0) : (360.0 / 60.0);
    double cprlat = raw_lat / 131072.0;
    double cprlon = raw_lon / 131072.0;

    // Latitude
    int j = (int)floor(ref_lat / d_lat) +
            (int)floor(0.5 + fmod_pos(ref_lat, d_lat) / d_lat - cprlat);
    double rlat = d_lat * (j + cprlat);

    // Sanity check latitude
    if (rlat < -90 || rlat > 90) return 0;

    // Longitude
    int nl = NL(rlat);
    int ni = fflag ? ((nl - 1) > 1 ? (nl - 1) : 1) : (nl > 1 ? nl : 1);
    double d_lon = 360.0 / ni;

    int m = (int)floor(ref_lon / d_lon) +
            (int)floor(0.5 + fmod_pos(ref_lon, d_lon) / d_lon - cprlon);
    double rlon = d_lon * (m + cprlon);

    if (rlon >= 180) rlon -= 360;
    if (rlon < -180) rlon += 360;

    // Sanity: reject if decoded position is >50 nm from reference
    // (local decode is valid within ~180 nm, but 50 nm catches errors early)
    double dlat = rlat - ref_lat;
    double dlon = (rlon - ref_lon) * cos(ref_lat * M_PI / 180.0);
    double dist_nm = sqrt(dlat * dlat + dlon * dlon) * 60.0;
    if (dist_nm > 50.0) return 0;

    *out_lat = rlat;
    *out_lon = rlon;
    return 1;
}

// ============================================================
// Aircraft state table
// ============================================================

typedef struct {
    uint32_t icao;
    char callsign[9];
    int altitude;
    int speed;
    int heading;
    int vert_rate;
    double lat, lon;
    int squawk;
    int has_position;
    int64_t last_seen;
    int active;
    uint32_t msg_count;
    // Position validation
    double good_lat, good_lon;  // last accepted (non-suspect) position
    int64_t good_pos_ts;        // timestamp of last good position
    int position_suspect;       // 1 = current lat/lon may be wrong, awaiting recovery
    int suspect_count;          // consecutive suspect events (give up after N)
    // Consensus recovery: track agreeing global decodes during suspect mode
    double suspect_lat, suspect_lon;  // last candidate during recovery
    int suspect_agree;                // consecutive globals agreeing with each other
} aircraft_t;

static aircraft_t aircraft_table[AIRCRAFT_TABLE_SIZE];
static SemaphoreHandle_t aircraft_mutex = NULL;

static aircraft_t *aircraft_get(uint32_t icao) {
    for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++)
        if (aircraft_table[i].active && aircraft_table[i].icao == icao)
            return &aircraft_table[i];

    for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++) {
        if (!aircraft_table[i].active) {
            memset(&aircraft_table[i], 0, sizeof(aircraft_t));
            aircraft_table[i].icao = icao;
            aircraft_table[i].active = 1;
            return &aircraft_table[i];
        }
    }

    // Evict oldest
    int oldest = 0;
    for (int i = 1; i < AIRCRAFT_TABLE_SIZE; i++)
        if (aircraft_table[i].last_seen < aircraft_table[oldest].last_seen)
            oldest = i;
    memset(&aircraft_table[oldest], 0, sizeof(aircraft_t));
    aircraft_table[oldest].icao = icao;
    aircraft_table[oldest].active = 1;
    return &aircraft_table[oldest];
}

static void aircraft_expire(void) {
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++)
        if (aircraft_table[i].active &&
            (now - aircraft_table[i].last_seen) > AIRCRAFT_MAX_AGE_US)
            aircraft_table[i].active = 0;
}

// Check if a candidate position is a plausible jump from the last known good position.
// Returns 1 if plausible, 0 if implausible.
static int cpr_jump_check(aircraft_t *ac, double cand_lat, double cand_lon, int64_t now) {
    if (!ac->good_pos_ts) return 1;  // no reference — first position, always accept

    double elapsed_s = (double)(now - ac->good_pos_ts) / 1000000.0;
    if (elapsed_s < 0.1) elapsed_s = 0.1;  // avoid division by zero

    double max_nm = CPR_JUMP_FLOOR_NM + elapsed_s * CPR_JUMP_RATE_NM_S;

    double dlat = cand_lat - ac->good_lat;
    double dlon = (cand_lon - ac->good_lon) * cos(ac->good_lat * M_PI / 180.0);
    double dist_nm = sqrt(dlat * dlat + dlon * dlon) * 60.0;

    return dist_nm <= max_nm;
}

// ============================================================
// Receiver position (fed by GPS task)
// ============================================================

static receiver_pos_t s_rx_pos = {0};
static SemaphoreHandle_t s_rx_pos_mutex = NULL;

static void rx_pos_init(void) {
    if (s_rx_pos_mutex == NULL)
        s_rx_pos_mutex = xSemaphoreCreateMutex();
}

void adsb_set_receiver_pos(double lat, double lon, double alt_m,
                          int sats, double hdop, int fix_quality) {
    if (!s_rx_pos_mutex) return;
    xSemaphoreTake(s_rx_pos_mutex, portMAX_DELAY);
    s_rx_pos.lat = lat;
    s_rx_pos.lon = lon;
    s_rx_pos.alt_m = alt_m;
    s_rx_pos.fix_valid = 1;
    s_rx_pos.sats = sats;
    s_rx_pos.hdop = hdop;
    s_rx_pos.fix_quality = fix_quality;
    xSemaphoreGive(s_rx_pos_mutex);
}

receiver_pos_t adsb_get_receiver_pos(void) {
    receiver_pos_t p = {0};
    if (!s_rx_pos_mutex) return p;
    xSemaphoreTake(s_rx_pos_mutex, portMAX_DELAY);
    p = s_rx_pos;
    xSemaphoreGive(s_rx_pos_mutex);
    return p;
}

void adsb_set_bias_tee(bool on) {
    if (rtldev) {
        rtlsdr_set_bias_tee(rtldev, on ? 1 : 0);
        ESP_LOGI(TAG, "Bias-T: %s", on ? "ON" : "OFF");
    }
}

// Forward declaration — defined below
static void haversine(double lat1, double lon1, double lat2, double lon2,
                      double *out_dist_km, double *out_bearing_deg);

// Max range — updated by adsb_get_stats(), referenced by gain logs
static volatile double s_max_range_nm = 0.0;

adsb_stats_t adsb_get_stats(void) {
    adsb_stats_t stats = {0};
    stats.total_messages = s_total_messages;
    stats.msg_rate = s_msg_rate;
    stats.rtlsdr_connected = s_rtlsdr_connected;
    stats.rtlsdr_error = s_rtlsdr_error;

    // Count active aircraft
    int count = 0;
    int64_t now = esp_timer_get_time();
    if (aircraft_mutex) {
        xSemaphoreTake(aircraft_mutex, portMAX_DELAY);
        for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++) {
            if (aircraft_table[i].active &&
                (now - aircraft_table[i].last_seen) <= AIRCRAFT_MAX_AGE_US)
                count++;
        }
        xSemaphoreGive(aircraft_mutex);
    }
    stats.active_aircraft = count;

    // Find nearest and farthest aircraft
    stats.nearest_icao = 0;
    stats.nearest_dist_nm = 0;
    stats.farthest_dist_nm = 0;
    stats.farthest_icao = 0;
    receiver_pos_t rx = adsb_get_receiver_pos();
    if (rx.fix_valid && aircraft_mutex) {
        double best_dist = 1e9;
        double worst_dist = 0;
        xSemaphoreTake(aircraft_mutex, portMAX_DELAY);
        for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++) {
            if (!aircraft_table[i].active) continue;
            if (!aircraft_table[i].has_position) continue;
            if ((now - aircraft_table[i].last_seen) > AIRCRAFT_MAX_AGE_US) continue;
            double dist_km, brg;
            haversine(rx.lat, rx.lon, aircraft_table[i].lat, aircraft_table[i].lon, &dist_km, &brg);
            if (dist_km < best_dist) {
                best_dist = dist_km;
                stats.nearest_icao = aircraft_table[i].icao;
                stats.nearest_dist_nm = dist_km * 0.539957;
                stats.nearest_alt = aircraft_table[i].altitude;
                if (aircraft_table[i].callsign[0]) {
                    strncpy(stats.nearest_callsign, aircraft_table[i].callsign, 8);
                    stats.nearest_callsign[8] = '\0';
                }
            }
            if (dist_km > worst_dist) {
                worst_dist = dist_km;
                stats.farthest_icao = aircraft_table[i].icao;
                stats.farthest_dist_nm = dist_km * 0.539957;
            }
        }
        xSemaphoreGive(aircraft_mutex);
    }

    stats.gain_tenths = R820T_GAINS[s_gain_idx];
    stats.gain_auto = s_gain_auto;
    stats.gain_phase = s_gain_phase;
    stats.crc_error_rate = s_gain_error_rate;
    stats.pos_rate = s_gain_pos_rate_ema;
    stats.avg_signal = s_gain_avg_signal;
    stats.avg_delta = s_gain_avg_delta;
    s_max_range_nm = stats.farthest_dist_nm;

    /* Build human-readable dongle model string for UI/MQTT */
    stats.dongle_model[0] = '\0';
    if (rtldev) {
        rtlsdr_get_dongle_info(rtldev, stats.dongle_model, sizeof(stats.dongle_model),
                               NULL, NULL);
    } else {
        snprintf(stats.dongle_model, sizeof(stats.dongle_model), "not connected");
    }

    return stats;
}

// ── Gain control ──────────────────────────────────────────────────────────

// Find the closest valid R820T gain index for a given tenths-of-dB value
static int gain_find_idx(uint16_t tenths) {
    int best = 0;
    int best_diff = abs((int)tenths - (int)R820T_GAINS[0]);
    for (int i = 1; i < (int)R820T_GAIN_COUNT; i++) {
        int diff = abs((int)tenths - (int)R820T_GAINS[i]);
        if (diff < best_diff) { best_diff = diff; best = i; }
    }
    return best;
}

// Apply the current gain index to the hardware
static void gain_apply(void) {
    if (rtldev) {
        rtlsdr_set_tuner_gain(rtldev, R820T_GAINS[s_gain_idx]);
    }
}

// Set gain mode and value from external callers (serial console, settings)
void adsb_set_gain(int mode, uint16_t gain_tenths) {
    if (mode == 0) {
        // Auto (adaptive) — restart from phase 1
        s_gain_auto = true;
        s_gain_phase = 1;
        s_gain_stable_count = 0;
        s_gain_p1_prev_pos_rate = 0.0f;
        s_gain_p1_prev_ok = 0;
        s_gain_p1_last_was_up = false;
        s_gain_p1_up_suppress = 0;
        s_gain_warmup = true;
        s_gain_err_ema = 0.0f;
        s_gain_pos_rate_ema = 0.0f;
        s_gain_cooldown = 0;
        s_gain_step_dir = 0;
        s_gain_ceiling = R820T_GAIN_COUNT;
        s_gain_low_err_count = 0;
        s_gain_ceiling_strikes = 0;
        s_gain_ceiling_age = 0;
        s_gain_floor = -1;
        s_gain_high_err_count = 0;
        s_gain_floor_strikes = 0;
        s_gain_floor_age = 0;
        s_gain_win_pos = 0;
        memset((void *)s_gain_profile, 0, sizeof(s_gain_profile));
        memset((void *)s_gain_profile_samples, 0, sizeof(s_gain_profile_samples));
        s_gain_idx = R820T_GAIN_COUNT - 1; // start at max
        s_gain_pending = true;  // deferred — USB task will apply
        g_settings.adsb_gain_mode = 0;
        char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
        serial_console_print("%sGAIN: Switched to adaptive gain (starting at %.1f dB, phase 1)\n",
                             _ts, R820T_GAINS[s_gain_idx] / 10.0);
    } else {
        // Manual (fixed)
        s_gain_auto = false;
        s_gain_idx = gain_find_idx(gain_tenths);
        g_settings.adsb_gain_mode = 1;
        g_settings.adsb_gain_tenths = R820T_GAINS[s_gain_idx];
        s_gain_pending = true;  // deferred — USB task will apply
        char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
        serial_console_print("%sGAIN: Set manual gain: %.1f dB (step %d/%d)\n",
                             _ts, R820T_GAINS[s_gain_idx] / 10.0,
                             s_gain_idx, (int)R820T_GAIN_COUNT - 1);
    }
    g_settings_save_pending = true;
}

// Restart adaptive gain from current level — re-enters P1 without resetting to max.
// Clears ceiling/floor/EMA (stale environmental context) but keeps the current gain
// setting. Use after antenna changes or environment shifts.
void adsb_gain_restart(void) {
    if (!s_gain_auto) return;  // only meaningful in auto mode

    int old_idx = s_gain_idx;  // preserve current gain
    s_gain_phase = 1;
    s_gain_stable_count = 0;
    s_gain_p1_prev_pos_rate = 0.0f;
    s_gain_p1_prev_ok = 0;
    s_gain_p1_last_was_up = false;
    s_gain_p1_up_suppress = 0;
    s_gain_warmup = false;  // no warmup — decoder is already running
    s_gain_err_ema = 0.0f;
    s_gain_pos_rate_ema = 0.0f;
    s_gain_cooldown = 0;
    s_gain_step_dir = 0;
    s_gain_ceiling = R820T_GAIN_COUNT;
    s_gain_low_err_count = 0;
    s_gain_ceiling_strikes = 0;
    s_gain_ceiling_age = 0;
    s_gain_floor = -1;
    s_gain_high_err_count = 0;
    s_gain_floor_strikes = 0;
    s_gain_floor_age = 0;
    s_gain_win_pos = 0;
    // Keep gain profile — historical data is still useful
    // Keep s_gain_idx — don't reset to max

    char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
    serial_console_print("%sGAIN: Restart from %.1f dB — re-entering phase 1\n",
                         _ts, R820T_GAINS[old_idx] / 10.0);
}

// Target dB reductions for phase 1 proportional response (in tenths of dB).
// These are consistent power reductions regardless of where we are in the
// non-uniform R820T gain table.  dB is logarithmic: 3 dB = ½ power.
#define GAIN_DB_DROP_EXTREME  60   // >90% error: ~6 dB (power ÷ 4)
#define GAIN_DB_DROP_SEVERE   45   // 80-90%: ~4.5 dB (power ÷ 2.8)
#define GAIN_DB_DROP_MODERATE 30   // 60-80%: ~3 dB (power ÷ 2)
#define GAIN_DB_DROP_MILD     15   // 50-60%: ~1.5 dB (power ÷ 1.4)

// Phase 2 step-up dB targets — graduated by how far below target band
#define GAIN_DB_RAISE_MILD     10  // 30-40% EMA: +1.0 dB
#define GAIN_DB_RAISE_MODERATE 15  // 20-30% EMA: +1.5 dB
#define GAIN_DB_RAISE_STRONG   25  // 10-20% EMA: +2.5 dB
#define GAIN_DB_RAISE_FULL     30  // ≤10% EMA:   +3.0 dB

// Find the gain index closest to (current - reduction_tenths), clamped to floor.
// Returns the index whose gain value is nearest to the target without going
// below GAIN_FLOOR_IDX.  Always moves at least 1 step if above floor.
static int gain_idx_for_db_drop(int current_idx, int reduction_tenths) {
    int target_val = R820T_GAINS[current_idx] - reduction_tenths;
    if (target_val < (int)R820T_GAINS[GAIN_FLOOR_IDX]) target_val = R820T_GAINS[GAIN_FLOOR_IDX];

    int best_idx = GAIN_FLOOR_IDX;
    int best_dist = abs((int)R820T_GAINS[GAIN_FLOOR_IDX] - target_val);
    for (int i = GAIN_FLOOR_IDX + 1; i < current_idx; i++) {
        int dist = abs((int)R820T_GAINS[i] - target_val);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }
    // Ensure we always move at least 1 step
    if (best_idx >= current_idx && current_idx > GAIN_FLOOR_IDX) {
        best_idx = current_idx - 1;
    }
    return best_idx;
}

// Find the gain index closest to (current + raise_tenths), clamped to max.
// Returns the index whose gain value is nearest to the target without going
// above R820T_GAIN_COUNT-1.  Always moves at least 1 step if below max.
static int gain_idx_for_db_raise(int current_idx, int raise_tenths) {
    int max_idx = (int)R820T_GAIN_COUNT - 1;
    int target_val = R820T_GAINS[current_idx] + raise_tenths;
    if (target_val > (int)R820T_GAINS[max_idx]) target_val = R820T_GAINS[max_idx];

    int best_idx = max_idx;
    int best_dist = abs((int)R820T_GAINS[max_idx] - target_val);
    for (int i = current_idx + 1; i < max_idx; i++) {
        int dist = abs((int)R820T_GAINS[i] - target_val);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }
    // Ensure we always move at least 1 step
    if (best_idx <= current_idx && current_idx < max_idx) {
        best_idx = current_idx + 1;
    }
    return best_idx;
}

// Throughput-optimized adaptive gain evaluation
// Primary metric: position updates per second (the actual useful output)
// Phase 1: graduated dB-targeted response — consistent power reductions
//   >90%: -6 dB, 80-90%: -4.5 dB, 67-80%: -3 dB, 50-67%: -1.5 dB (throughput-gated)
//   Target band: 40-50% error. Below 40%: step up (accelerated if confirmed).
// Phase 2: EMA smoothing, graduated dB-targeted step-ups, probe-and-compare
//   Emergency brake at >67% EMA → fall back to phase 1
static void gain_evaluate(mode_s_t *st) {
    uint32_t ok   = st->stat_crc_ok;
    uint32_t fail = st->stat_crc_fail;
    uint32_t total = ok + fail;
    uint32_t pos  = s_gain_win_pos;

    // Read signal quality metrics (accumulated for CRC-ok messages only)
    uint32_t avg_signal = ok > 0 ? (uint32_t)(st->stat_signal_sum / ok) : 0;
    uint32_t avg_delta  = ok > 0 ? (uint32_t)(st->stat_delta_sum / ok) : 0;

    // Reset counters for next window
    st->stat_crc_ok = 0;
    st->stat_crc_fail = 0;
    st->stat_preambles = 0;
    st->stat_signal_sum = 0;
    st->stat_delta_sum = 0;
    s_gain_win_pos = 0;

    // ── Minimum message guards ──
    if (total == 0) {
        // Zero messages: might be deaf, or just no traffic.
        // P1: step up — try to find signal. P2: hold — we were working, wait it out.
        if (s_gain_phase == 1 && s_gain_idx + 1 < (int)R820T_GAIN_COUNT
            && s_gain_idx + 1 < s_gain_ceiling) {
            s_gain_idx++;
            s_gain_stable_count = 0;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P1] Up: %.1f dB (no messages — increasing sensitivity)\n",
                                 _ts, R820T_GAINS[s_gain_idx] / 10.0);
        }
        return;
    }
    if (total < GAIN_MIN_SAMPLES) return;  // too few messages for reliable evaluation

    float err_rate = (float)fail / (float)total;

    // Compute window duration for rate calculations
    int64_t now = esp_timer_get_time();
    float win_secs = (float)(now - s_gain_last_eval) / 1000000.0f;
    if (win_secs < 1.0f) win_secs = 1.0f;  // avoid division by zero
    float pos_rate = (float)pos / win_secs;

    int old_idx = s_gain_idx;

    // Store signal quality for stats API
    s_gain_avg_signal = avg_signal;
    s_gain_avg_delta = avg_delta;

    if (s_gain_phase == 1) {
        // ══════════════════════════════════════════════════════════
        // Phase 1: Fast convergence — dB-targeted proportional response
        // Uses consistent power reductions regardless of non-uniform gain table.
        // >90%: -6 dB (power ÷ 4)
        // 80-90%: -4.5 dB (power ÷ 2.8)
        // 67-80%: -3 dB (power ÷ 2)
        // 50-67%: -1.5 dB (gentle, with throughput check)
        // 40-50%: target range — sweet spot for coverage (~45% ideal)
        // <40%: step up — confirmed step-ups accelerate to +1.5 dB
        //        step-ups that hurt throughput → hold (overshot)
        // Converge: 3 consecutive windows without a gain change
        // ══════════════════════════════════════════════════════════
        s_gain_error_rate = err_rate;
        bool changed = false;
        if (s_gain_p1_up_suppress > 0) s_gain_p1_up_suppress--;

        // Signal quality gate: only reduce gain if we have enough CRC-ok
        // messages with strong signal levels to confirm ADC saturation.
        // Weak/absent signals + high error = noise or no traffic.
        bool can_reduce = (ok >= GAIN_MIN_OK_FOR_STATS
                           && avg_signal >= GAIN_MIN_SIG_FOR_DOWN);

        if (err_rate > 0.90f && s_gain_idx > GAIN_FLOOR_IDX && can_reduce) {
            // Near-total garbage — drop ~6 dB (power ÷ 4)
            int new_idx = gain_idx_for_db_drop(s_gain_idx, GAIN_DB_DROP_EXTREME);
            s_gain_idx = new_idx;
            changed = true;
            s_gain_p1_last_was_up = false;
            s_gain_p1_up_suppress = 0;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P1] Down: %.1f → %.1f dB (−%.1f dB, err=%.0f%%, pos=%.1f/s, ok=%lu, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                 (R820T_GAINS[old_idx] - R820T_GAINS[s_gain_idx]) / 10.0,
                                 err_rate * 100, pos_rate, (unsigned long)ok,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta);
        } else if (err_rate > 0.80f && s_gain_idx > GAIN_FLOOR_IDX && can_reduce) {
            // Severe clipping — drop ~4.5 dB (power ÷ 2.8)
            int new_idx = gain_idx_for_db_drop(s_gain_idx, GAIN_DB_DROP_SEVERE);
            s_gain_idx = new_idx;
            changed = true;
            s_gain_p1_last_was_up = false;
            s_gain_p1_up_suppress = 0;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P1] Down: %.1f → %.1f dB (−%.1f dB, err=%.0f%%, pos=%.1f/s, ok=%lu, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                 (R820T_GAINS[old_idx] - R820T_GAINS[s_gain_idx]) / 10.0,
                                 err_rate * 100, pos_rate, (unsigned long)ok,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta);
        } else if (err_rate > GAIN_ERR_SEVERE && s_gain_idx > GAIN_FLOOR_IDX && can_reduce) {
            // Significant clipping (67-80%) — drop ~3 dB
            int new_idx = gain_idx_for_db_drop(s_gain_idx, GAIN_DB_DROP_MODERATE);
            s_gain_idx = new_idx;
            changed = true;
            s_gain_p1_last_was_up = false;
            s_gain_p1_up_suppress = 0;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P1] Down: %.1f → %.1f dB (−%.1f dB, err=%.0f%%, pos=%.1f/s, ok=%lu, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                 (R820T_GAINS[old_idx] - R820T_GAINS[s_gain_idx]) / 10.0,
                                 err_rate * 100, pos_rate, (unsigned long)ok,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta);
        } else if (err_rate > gain_err_high() && s_gain_idx > GAIN_FLOOR_IDX && can_reduce) {
            // Saturation starting (50-67%) — drop ~1.5 dB only if good message count holds
            // Fix: use ok count (good CRC messages) not pos_rate — pos_rate is confounded
            // by traffic changes (aircraft entering/leaving range between windows).
            if (s_gain_p1_prev_ok == 0 || ok >= (uint32_t)(s_gain_p1_prev_ok * 0.80f)) {
                int new_idx = gain_idx_for_db_drop(s_gain_idx, GAIN_DB_DROP_MILD);
                s_gain_idx = new_idx;
                changed = true;
                s_gain_p1_last_was_up = false;
                s_gain_p1_up_suppress = 0;
                gain_apply();
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P1] Down: %.1f → %.1f dB (−%.1f dB, err=%.0f%%, ok=%lu, prev_ok=%lu, pos=%.1f/s, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                     (R820T_GAINS[old_idx] - R820T_GAINS[s_gain_idx]) / 10.0,
                                     err_rate * 100, (unsigned long)ok, (unsigned long)s_gain_p1_prev_ok, pos_rate,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            } else {
                // Good messages dropped >20% — we're losing real aircraft, hold
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P1] Holding %.1f dB — ok dropped (err=%.0f%%, ok=%lu, prev_ok=%lu, pos=%.1f/s, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[s_gain_idx] / 10.0, err_rate * 100,
                                     (unsigned long)ok, (unsigned long)s_gain_p1_prev_ok, pos_rate,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
                s_gain_p1_last_was_up = false;
            }
        } else if (err_rate < gain_err_low() && s_gain_idx < (int)R820T_GAIN_COUNT - 1
                   && s_gain_p1_up_suppress == 0) {
            // Low error (<40%) — we're deaf to distant aircraft, step up
            // If last action was also a step-up that improved ok count and
            // we're still below the saturation threshold, take a 1.5 dB step.
            // Otherwise, conservative single-index step.
            if (s_gain_p1_last_was_up && s_gain_p1_prev_ok > 0
                && ok >= s_gain_p1_prev_ok) {
                // Confirmed: last step-up helped. Accelerate with 1.5 dB step.
                int new_idx = gain_idx_for_db_raise(s_gain_idx, GAIN_DB_DROP_MILD);
                s_gain_idx = new_idx;
                changed = true;
                s_gain_p1_last_was_up = true;
                gain_apply();
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P1] Up (accel): %.1f → %.1f dB (+%.1f dB, err=%.0f%%, ok=%lu, prev_ok=%lu, pos=%.1f/s, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                     (R820T_GAINS[s_gain_idx] - R820T_GAINS[old_idx]) / 10.0,
                                     err_rate * 100, (unsigned long)ok, (unsigned long)s_gain_p1_prev_ok, pos_rate,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            } else if (s_gain_p1_last_was_up && s_gain_p1_prev_ok > 0
                       && ok < (uint32_t)(s_gain_p1_prev_ok * 0.80f)) {
                // Last step-up hurt throughput — we overshot
                // Suppress further step-ups to allow convergence at this gain
                s_gain_p1_up_suppress = GAIN_P1_STABLE_WIN;
                s_gain_p1_last_was_up = false;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P1] Step-up hurt throughput — suppressing (err=%.0f%%, ok=%lu, prev_ok=%lu, pos=%.1f/s, sig=%lu delta=%lu)\n",
                                     _ts, err_rate * 100, (unsigned long)ok, (unsigned long)s_gain_p1_prev_ok, pos_rate,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            } else {
                // First step-up or throughput ambiguous — conservative single step
                s_gain_idx++;
                changed = true;
                s_gain_p1_last_was_up = true;
                gain_apply();
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P1] Up: %.1f → %.1f dB (+%.1f dB, err=%.0f%%, ok=%lu, prev_ok=%lu, pos=%.1f/s, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                     (R820T_GAINS[s_gain_idx] - R820T_GAINS[old_idx]) / 10.0,
                                     err_rate * 100, (unsigned long)ok, (unsigned long)s_gain_p1_prev_ok, pos_rate,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            }
        }
        // else: 40-50% error — target range, no change needed

        // Log when high error was blocked by weak signal gate
        if (!changed && !can_reduce && err_rate > gain_err_high()
            && s_gain_idx > GAIN_FLOOR_IDX) {
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P1] Holding %.1f dB — err=%.0f%% but weak signals (ok=%lu, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[s_gain_idx] / 10.0, err_rate * 100,
                                 (unsigned long)ok, (unsigned long)avg_signal, (unsigned long)avg_delta);
        }

        // Track throughput for next window's comparison
        s_gain_p1_prev_pos_rate = pos_rate;
        s_gain_p1_prev_ok = ok;

        if (changed) {
            s_gain_stable_count = 0;
        } else {
            s_gain_stable_count++;
            if (s_gain_stable_count >= GAIN_P1_STABLE_WIN) {
                // Converged — transition to phase 2
                s_gain_phase = 2;
                s_gain_err_ema = err_rate;
                s_gain_pos_rate_ema = pos_rate;
                s_gain_cooldown = 0;
                s_gain_step_dir = 0;
                s_gain_ceiling = R820T_GAIN_COUNT;
                s_gain_low_err_count = 0;
                s_gain_ceiling_strikes = 0;
                s_gain_ceiling_age = 0;
                s_gain_floor = -1;
                s_gain_high_err_count = 0;
                s_gain_floor_strikes = 0;
                s_gain_floor_age = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Converged at %.1f dB (err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu) → steady-state\n",
                                     _ts, R820T_GAINS[s_gain_idx] / 10.0, err_rate * 100, pos_rate, s_max_range_nm,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            }
        }

    } else {
        // ══════════════════════════════════════════════════════════
        // Phase 2: Steady-state — throughput-optimized
        // Primary: did we get more valid position updates?
        // Emergency: error rate as clipping detector only
        // ══════════════════════════════════════════════════════════

        // EMA update — only when we have enough CRC-ok messages for reliable
        // metrics. During no-traffic periods (ok < 3, noise triggers only), the
        // EMAs freeze at their last known-good values rather than drifting toward
        // noise-floor error rates. This prevents the algorithm from thinking
        // error is 90%+ when traffic returns after a quiet period.
        if (ok >= GAIN_MIN_OK_FOR_STATS) {
            s_gain_err_ema = GAIN_P2_EMA_ALPHA * err_rate + (1.0f - GAIN_P2_EMA_ALPHA) * s_gain_err_ema;
            s_gain_pos_rate_ema = GAIN_P2_EMA_ALPHA * pos_rate + (1.0f - GAIN_P2_EMA_ALPHA) * s_gain_pos_rate_ema;
        }
        s_gain_error_rate = s_gain_err_ema;

        // ── Update historical profile (phase 2 only — 60s windows are reliable) ──
        // Gate on ok count: don't teach the profile "this gain produces nothing"
        // when there's simply no traffic.
        int gi = s_gain_idx;
        if (ok >= GAIN_MIN_OK_FOR_STATS && gi >= 0 && gi < (int)R820T_GAIN_COUNT) {
            if (s_gain_profile_samples[gi] == 0) {
                s_gain_profile[gi] = pos_rate; // seed
            } else {
                s_gain_profile[gi] = GAIN_PROFILE_ALPHA * pos_rate
                                   + (1.0f - GAIN_PROFILE_ALPHA) * s_gain_profile[gi];
            }
            if (s_gain_profile_samples[gi] < 65535)
                s_gain_profile_samples[gi]++;
        }

        // ── Emergency brake: severe error, BUT only if signals confirm clipping ──
        // High EMA error + strong signals = genuine ADC saturation → re-enter P1.
        // High EMA error + weak/absent signals = noise or no traffic → hold in P2.
        // Without this gate, traffic disappearing causes a false "clipping" diagnosis
        // that triggers P1 re-entry, oscillation, and the nighttime death spiral.
        if (s_gain_err_ema > GAIN_ERR_SEVERE) {
            bool evidence_of_clipping = (ok >= GAIN_MIN_OK_FOR_STATS
                                         && avg_signal >= GAIN_MIN_SIG_FOR_CLIPPING);
            if (evidence_of_clipping) {
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Confirmed clipping (EMA err=%.0f%%, sig=%lu delta=%lu) → returning to fast convergence\n",
                                     _ts, s_gain_err_ema * 100,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
                s_gain_phase = 1;
                s_gain_stable_count = 0;
                s_gain_p1_prev_pos_rate = 0.0f;
                s_gain_p1_prev_ok = 0;
                s_gain_p1_last_was_up = false;
                s_gain_p1_up_suppress = 0;
                s_gain_ceiling = R820T_GAIN_COUNT;
                s_gain_low_err_count = 0;
                s_gain_ceiling_strikes = 0;
                s_gain_ceiling_age = 0;
                s_gain_floor = -1;
                s_gain_high_err_count = 0;
                s_gain_floor_strikes = 0;
                s_gain_floor_age = 0;
                s_gain_step_dir = 0;
                return;
            }
            // else: EMA err is high but signals are weak/absent — not clipping.
            // Don't log every window (too noisy) — the P2 heartbeat shows the state.
        }

        // ── Soft ceiling expiry: sustained low error OR hard time cap ──
        if (s_gain_ceiling < (int)R820T_GAIN_COUNT) {
            s_gain_ceiling_age++;

            // Hard time cap — always retry after 20 minutes regardless
            if (s_gain_ceiling_age >= GAIN_CEILING_MAX_WIN) {
                s_gain_ceiling = R820T_GAIN_COUNT;
                s_gain_low_err_count = 0;
                s_gain_ceiling_strikes = 0;
                s_gain_ceiling_age = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Ceiling expired (age=%d windows) → will retry higher\n",
                                     _ts, GAIN_CEILING_MAX_WIN);
            }
        }
        // Early-out: sustained low error clears ceiling sooner
        if (s_gain_err_ema < gain_err_very_low()) {
            s_gain_low_err_count++;
            if (s_gain_low_err_count >= GAIN_CEILING_CLEAR_WIN &&
                s_gain_ceiling < (int)R820T_GAIN_COUNT) {
                s_gain_ceiling = R820T_GAIN_COUNT;
                s_gain_low_err_count = 0;
                s_gain_ceiling_strikes = 0;
                s_gain_ceiling_age = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Ceiling cleared (EMA err=%.1f%% × %d windows) → will retry higher\n",
                                     _ts, s_gain_err_ema * 100, GAIN_CEILING_CLEAR_WIN);
            }
        } else {
            s_gain_low_err_count = 0;
        }

        // ── Soft floor expiry: sustained high error OR hard time cap ──
        if (s_gain_floor >= 0) {
            s_gain_floor_age++;

            // Hard time cap — always retry after 20 minutes regardless
            if (s_gain_floor_age >= GAIN_FLOOR_MAX_WIN) {
                s_gain_floor = -1;
                s_gain_high_err_count = 0;
                s_gain_floor_strikes = 0;
                s_gain_floor_age = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Floor expired (age=%d windows) → will retry lower\n",
                                     _ts, GAIN_FLOOR_MAX_WIN);
            }
        }
        // Early-out: sustained high error clears floor sooner (saturation is back)
        if (s_gain_err_ema > gain_err_very_high()) {
            s_gain_high_err_count++;
            if (s_gain_high_err_count >= GAIN_FLOOR_CLEAR_WIN &&
                s_gain_floor >= 0) {
                s_gain_floor = -1;
                s_gain_high_err_count = 0;
                s_gain_floor_strikes = 0;
                s_gain_floor_age = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: Floor cleared (EMA err=%.1f%% × %d windows) → will retry lower\n",
                                     _ts, s_gain_err_ema * 100, GAIN_FLOOR_CLEAR_WIN);
            }
        } else {
            s_gain_high_err_count = 0;
        }

        // ── Cooldown after gain change ──
        if (s_gain_cooldown > 0) {
            s_gain_cooldown--;
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P2] Settling: %.1f dB, EMA err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu (cooldown %d)\n",
                                 _ts, R820T_GAINS[s_gain_idx] / 10.0,
                                 s_gain_err_ema * 100, s_gain_pos_rate_ema,
                                 s_max_range_nm, (unsigned long)avg_signal, (unsigned long)avg_delta,
                                 s_gain_cooldown);
            return;
        }

        // ── Throughput comparison: evaluate result of last gain step ──
        if (s_gain_step_dir != 0 && s_gain_pre_step_pos_rate > 0.0f) {
            float change = (s_gain_pos_rate_ema - s_gain_pre_step_pos_rate) / s_gain_pre_step_pos_rate;

            if (change < -GAIN_P2_THROUGHPUT_DROP) {
                // Throughput dropped >15%. But was it the gain step, or traffic change?
                // Compare signal environment: if avg_signal shifted dramatically,
                // the traffic mix changed (aircraft arrived/departed) and the
                // throughput drop isn't attributable to the gain step.
                bool traffic_shifted = false;
                if (s_gain_pre_step_signal > 0 && ok >= GAIN_MIN_OK_FOR_STATS) {
                    float sig_change = fabsf((float)avg_signal - (float)s_gain_pre_step_signal)
                                       / (float)s_gain_pre_step_signal;
                    traffic_shifted = (sig_change > GAIN_P2_SIG_CHANGE_THRESH);
                }

                if (traffic_shifted) {
                    // Signal environment changed >40% — throughput drop is likely from
                    // traffic change, not the gain step. Hold current gain, no strike.
                    s_gain_step_dir = 0;
                    s_gain_cooldown = GAIN_P2_COOLDOWN;
                    char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                    serial_console_print("%sGAIN: [P2] Holding %.1f dB — throughput %+.0f%% but traffic shifted (sig %lu → %lu, no strike)\n",
                                         _ts, R820T_GAINS[s_gain_idx] / 10.0, change * 100,
                                         (unsigned long)s_gain_pre_step_signal, (unsigned long)avg_signal);
                    return;
                }

                // Signal environment is similar — throughput drop is real, revert
                int bad_idx = s_gain_idx;
                if (s_gain_step_dir > 0) {
                    // Stepped up and lost throughput → ceiling strike + step back
                    s_gain_ceiling_strikes++;
                    if (s_gain_ceiling_strikes >= GAIN_CEILING_STRIKES) {
                        s_gain_ceiling = s_gain_idx;
                        s_gain_ceiling_age = 0;
                    }
                    s_gain_idx--;
                } else {
                    // Stepped down and lost throughput → floor strike + step back up
                    s_gain_floor_strikes++;
                    if (s_gain_floor_strikes >= GAIN_FLOOR_STRIKES) {
                        s_gain_floor = s_gain_idx;
                        s_gain_floor_age = 0;
                    }
                    if (s_gain_idx < (int)R820T_GAIN_COUNT - 1) s_gain_idx++;
                }
                s_gain_cooldown = GAIN_P2_COOLDOWN;
                s_gain_step_dir = 0;
                gain_apply();
                // Build status suffix
                const char *sfx = "";
                if (s_gain_ceiling < (int)R820T_GAIN_COUNT) sfx = ", ceiling set";
                else if (s_gain_ceiling_strikes > 0) sfx = ", ceiling strike";
                else if (s_gain_floor >= 0) sfx = ", floor set";
                else if (s_gain_floor_strikes > 0) sfx = ", floor strike";
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P2] Reverted: %.1f → %.1f dB (throughput %+.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu%s)\n",
                                     _ts, R820T_GAINS[bad_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                     change * 100, s_gain_pos_rate_ema, s_max_range_nm,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta, sfx);
                return;
            } else {
                // Throughput held or improved — step was good, reset strikes
                s_gain_ceiling_strikes = 0;
                s_gain_floor_strikes = 0;
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P2] Confirmed: %.1f dB (throughput %+.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[s_gain_idx] / 10.0, change * 100, s_gain_pos_rate_ema, s_max_range_nm,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
                s_gain_step_dir = 0;
                // Let the new level settle before profile gets a vote
                s_gain_cooldown = GAIN_P2_COOLDOWN;
                return;
            }
        }

        // ── Decide whether to try a gain step ──
        // Check if historical profile suggests a better gain level nearby.
        // Require both 5% relative AND 0.5/s absolute improvement —
        // at low throughput (<10/s) the absolute floor prevents noise probes.
        int best_neighbor = -1;
        float best_rate = s_gain_pos_rate_ema;
        float prof_threshold = best_rate * 1.05f;
        if (prof_threshold < best_rate + GAIN_PROFILE_MIN_DELTA)
            prof_threshold = best_rate + GAIN_PROFILE_MIN_DELTA;

        // Look one step up (if below ceiling and not at max)
        if (s_gain_idx + 1 < (int)R820T_GAIN_COUNT && s_gain_idx + 1 < s_gain_ceiling) {
            int up = s_gain_idx + 1;
            if (s_gain_profile_samples[up] >= GAIN_PROFILE_MIN_WIN
                && s_gain_profile[up] > prof_threshold) {
                best_neighbor = up;
                best_rate = s_gain_profile[up];
            }
        }
        // Look one step down (if above hard floor and soft floor)
        if (s_gain_idx - 1 >= GAIN_FLOOR_IDX && s_gain_idx - 1 > s_gain_floor) {
            int dn = s_gain_idx - 1;
            if (s_gain_profile_samples[dn] >= GAIN_PROFILE_MIN_WIN
                && s_gain_profile[dn] > prof_threshold) {
                best_neighbor = dn;
                best_rate = s_gain_profile[dn];
            }
        }

        // If profile suggests a better neighbor, probe it
        if (best_neighbor >= 0) {
            s_gain_pre_step_pos_rate = s_gain_pos_rate_ema;
            s_gain_pre_step_signal = avg_signal;
            s_gain_step_dir = (best_neighbor > s_gain_idx) ? 1 : -1;
            s_gain_idx = best_neighbor;
            s_gain_cooldown = GAIN_P2_COOLDOWN;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P2] Probing: %.1f → %.1f dB (profile: %.1f/s vs current %.1f/s, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                 best_rate, s_gain_pos_rate_ema,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta);
            return;
        }

        // No profile hint — use error rate heuristics for exploration
        if (s_gain_err_ema > gain_err_high() && s_gain_idx > GAIN_FLOOR_IDX
            && s_gain_idx - 1 > s_gain_floor
            && ok >= GAIN_MIN_OK_FOR_STATS && avg_signal >= GAIN_MIN_SIG_FOR_DOWN) {
            // Saturating (>50%) with strong signals — try stepping down
            s_gain_pre_step_pos_rate = s_gain_pos_rate_ema;
            s_gain_pre_step_signal = avg_signal;
            s_gain_step_dir = -1;
            s_gain_idx--;
            s_gain_cooldown = GAIN_P2_COOLDOWN;
            gain_apply();
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sGAIN: [P2] Exploring down: %.1f → %.1f dB (EMA err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu)\n",
                                 _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                 s_gain_err_ema * 100, s_gain_pos_rate_ema, s_max_range_nm,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta);
        } else if (s_gain_err_ema < gain_err_low()
                   && s_gain_idx < (int)R820T_GAIN_COUNT - 1
                   && s_gain_idx + 1 < s_gain_ceiling) {
            // Below target band — graduated step-up based on how far below
            int raise_tenths;
            if (s_gain_err_ema <= 0.10f) {
                raise_tenths = GAIN_DB_RAISE_FULL;      // ≤10%: +3.0 dB
            } else if (s_gain_err_ema <= 0.20f) {
                raise_tenths = GAIN_DB_RAISE_STRONG;     // 10-20%: +2.5 dB
            } else if (s_gain_err_ema <= 0.30f) {
                raise_tenths = GAIN_DB_RAISE_MODERATE;   // 20-30%: +1.5 dB
            } else {
                raise_tenths = GAIN_DB_RAISE_MILD;       // 30-40%: +1.0 dB
            }
            int new_idx = gain_idx_for_db_raise(s_gain_idx, raise_tenths);
            // Clamp to ceiling
            if (new_idx >= s_gain_ceiling) new_idx = s_gain_ceiling - 1;
            if (new_idx <= s_gain_idx) new_idx = s_gain_idx + 1; // always move at least 1
            if (new_idx < (int)R820T_GAIN_COUNT && new_idx < s_gain_ceiling) {
                s_gain_pre_step_pos_rate = s_gain_pos_rate_ema;
                s_gain_pre_step_signal = avg_signal;
                s_gain_step_dir = 1;
                s_gain_idx = new_idx;
                s_gain_cooldown = GAIN_P2_COOLDOWN;
                gain_apply();
                char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                serial_console_print("%sGAIN: [P2] Exploring up: %.1f → %.1f dB (+%.1f dB, EMA err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu)\n",
                                     _ts, R820T_GAINS[old_idx] / 10.0, R820T_GAINS[s_gain_idx] / 10.0,
                                     (R820T_GAINS[s_gain_idx] - R820T_GAINS[old_idx]) / 10.0,
                                     s_gain_err_ema * 100, s_gain_pos_rate_ema, s_max_range_nm,
                                     (unsigned long)avg_signal, (unsigned long)avg_delta);
            }
        }

        // ── P2 heartbeat: always log current state for webapp visibility ──
        {
            extern device_settings_t g_settings;
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            const char *bounds = "";
            if (s_gain_ceiling < (int)R820T_GAIN_COUNT && s_gain_floor >= 0)
                bounds = ", bounds: ceiling+floor set";
            else if (s_gain_ceiling < (int)R820T_GAIN_COUNT)
                bounds = ", bounds: ceiling set";
            else if (s_gain_floor >= 0)
                bounds = ", bounds: floor set";
            serial_console_print("%sGAIN: [P2] Steady: %.1f dB, EMA err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu, band=%d-%d%%%s\n",
                                 _ts, R820T_GAINS[s_gain_idx] / 10.0,
                                 s_gain_err_ema * 100, s_gain_pos_rate_ema,
                                 s_max_range_nm,
                                 (unsigned long)avg_signal, (unsigned long)avg_delta,
                                 g_settings.gain_err_low, g_settings.gain_err_high,
                                 bounds);
        }
    }
}

// --- Sort state for on-device aircraft list ---
static adsb_sort_col_t s_sort_col = ADSB_SORT_DIST;
static bool s_sort_asc = true;

void adsb_set_sort(int col, bool ascending) {
    s_sort_col = (adsb_sort_col_t)col;
    s_sort_asc = ascending;
}

int adsb_format_aircraft_list(char *buf, int bufsize) {
    int pos = 0;
    int64_t now = esp_timer_get_time();
    receiver_pos_t rx = adsb_get_receiver_pos();

    if (!aircraft_mutex) {
        pos += snprintf(buf + pos, bufsize - pos, "Not initialized");
        return pos;
    }

    // Collect active aircraft with precomputed sort keys
    typedef struct { int idx; double dist_km; } ac_entry_t;
    ac_entry_t entries[AIRCRAFT_TABLE_SIZE];
    int count = 0;

    xSemaphoreTake(aircraft_mutex, portMAX_DELAY);
    for (int i = 0; i < AIRCRAFT_TABLE_SIZE; i++) {
        if (!aircraft_table[i].active) continue;
        if ((now - aircraft_table[i].last_seen) > AIRCRAFT_MAX_AGE_US) continue;
        entries[count].idx = i;
        entries[count].dist_km = 1e9;
        if (rx.fix_valid && aircraft_table[i].has_position) {
            double d, b;
            haversine(rx.lat, rx.lon, aircraft_table[i].lat, aircraft_table[i].lon, &d, &b);
            entries[count].dist_km = d;
        }
        count++;
    }

    // Insertion sort by selected column
    for (int i = 1; i < count; i++) {
        ac_entry_t key = entries[i];
        aircraft_t *ac_key = &aircraft_table[key.idx];
        int j = i - 1;
        while (j >= 0) {
            aircraft_t *ac_j = &aircraft_table[entries[j].idx];
            int cmp = 0;
            switch (s_sort_col) {
                case ADSB_SORT_ICAO:
                    cmp = (int)(ac_j->icao > ac_key->icao) - (int)(ac_j->icao < ac_key->icao);
                    break;
                case ADSB_SORT_CALL:
                    cmp = strcmp(ac_j->callsign, ac_key->callsign);
                    break;
                case ADSB_SORT_ALT:
                    cmp = (ac_j->altitude > ac_key->altitude) - (ac_j->altitude < ac_key->altitude);
                    break;
                case ADSB_SORT_SPD:
                    cmp = (ac_j->speed > ac_key->speed) - (ac_j->speed < ac_key->speed);
                    break;
                case ADSB_SORT_HDG:
                    cmp = (ac_j->heading > ac_key->heading) - (ac_j->heading < ac_key->heading);
                    break;
                case ADSB_SORT_DIST:
                default:
                    cmp = (entries[j].dist_km > key.dist_km) - (entries[j].dist_km < key.dist_km);
                    break;
            }
            if (!s_sort_asc) cmp = -cmp;
            if (cmp <= 0) break;
            entries[j + 1] = entries[j];
            j--;
        }
        entries[j + 1] = key;
    }

    if (count == 0) {
        pos += snprintf(buf + pos, bufsize - pos, "No aircraft");
    }

    for (int e = 0; e < count && pos < bufsize - 80; e++) {
        aircraft_t *ac = &aircraft_table[entries[e].idx];
        char call[9] = "--------";
        if (ac->callsign[0]) {
            snprintf(call, sizeof(call), "%-8s", ac->callsign);
        }

        double dist_nm = entries[e].dist_km * 0.539957;
        char dist_str[12] = "   --";
        if (rx.fix_valid && ac->has_position && entries[e].dist_km < 1e8) {
            snprintf(dist_str, sizeof(dist_str), "%5.1f", dist_nm);
        }

        char alt_str[12] = "  ----";
        if (ac->altitude != 0) {
            snprintf(alt_str, sizeof(alt_str), "%6d", ac->altitude);
        }

        char spd_str[12] = " ---";
        if (ac->speed > 0) {
            snprintf(spd_str, sizeof(spd_str), "%4d", ac->speed);
        }

        char hdg_str[12] = "---";
        if (ac->heading > 0) {
            snprintf(hdg_str, sizeof(hdg_str), "%3d", (int)ac->heading);
        }

        pos += snprintf(buf + pos, bufsize - pos,
            "%06lX %-8s%s %s %s %s\n",
            (unsigned long)ac->icao, call, alt_str, spd_str, hdg_str, dist_str);
    }
    xSemaphoreGive(aircraft_mutex);

    return count;
}

// Great-circle distance (km) and initial bearing (degrees) using haversine
static void haversine(double lat1, double lon1, double lat2, double lon2,
                      double *dist_km, double *bearing_deg) {
    double d2r = M_PI / 180.0;
    double rlat1 = lat1 * d2r, rlat2 = lat2 * d2r;
    double dlat = (lat2 - lat1) * d2r;
    double dlon = (lon2 - lon1) * d2r;

    double a = sin(dlat/2) * sin(dlat/2) +
               cos(rlat1) * cos(rlat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    *dist_km = 6371.0 * c;

    double y = sin(dlon) * cos(rlat2);
    double x = cos(rlat1) * sin(rlat2) - sin(rlat1) * cos(rlat2) * cos(dlon);
    double brg = atan2(y, x) * 180.0 / M_PI;
    if (brg < 0) brg += 360.0;
    *bearing_deg = brg;
}

#include "sd_logger.h"

// ============================================================
// SD card logging
// ============================================================

static sd_log_ch_t sd_ch;                    // shared channel handle
static int64_t sd_log_last_sync = 0;
#define SD_SYNC_INTERVAL_US  1000000LL       // flush every 1s if ≥1 sector
#define SD_SYNC_MAX_US       5000000LL       // hard max: flush every 5s regardless
#define SD_SYNC_MIN_BYTES    512             // min data for time-based flush

// PSRAM write buffer — messages accumulate here between flushes.
// File handle stays OPEN between sync cycles for performance.
// fsync() on every flush ensures data is committed to the card,
// so a hard reset loses at most one sync interval of buffered data.
#define SD_LOG_BUFSIZE  (64 * 1024)
static char *sd_log_buf = NULL;
static int   sd_log_buf_pos = 0;

static char sd_log_filename_buf[64];  // scratch for filename generation

static const char *sd_log_pick_filename(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);

    if (!time_manager_is_trusted_fn() || now < 1704067200LL) {
        // No GPS/NTP fix yet — use boot-numbered name.
        // Will be renamed to UTC timestamp when trusted time arrives.
        snprintf(sd_log_filename_buf, sizeof(sd_log_filename_buf),
            "/sdcard/logs/adsb_boot%s.csv", sd_config_boot_id());
    } else {
        strftime(sd_log_filename_buf, sizeof(sd_log_filename_buf),
            "/sdcard/logs/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);
    }
    return sd_log_filename_buf;
}

#define ADSB_CSV_HEADER \
    "timestamp_utc,raw_msg,icao,callsign,altitude_ft,speed_kt," \
    "heading_deg,vrate_fpm,lat,lon,squawk," \
    "rx_lat,rx_lon,range_km,bearing_deg," \
    "rx_sats,rx_hdop\n"

static void sd_log_create(void) {
    const char *fn = sd_log_pick_filename();
    if (!sd_log_ch_open(&sd_ch, fn, ADSB_CSV_HEADER)) return;
    sd_log_last_sync = esp_timer_get_time();
    // NOTE: sd_log_buf_pos intentionally NOT zeroed — the buffer may
    // contain valid CSV rows from before a failure.  The caller flushes
    // the buffer after create/reopen.
}

static void sd_log_flush(void) {
    if (!sd_ch.initialized || sd_log_buf_pos == 0) return;
    int n = sd_log_buf_pos;
    sd_log_ch_flush(&sd_ch, sd_log_buf, n);
    if (sd_ch.initialized) {
        // Flush succeeded — clear buffer
        sd_log_buf_pos = 0;
        sd_log_last_sync = esp_timer_get_time();
        sd_log_record_flush(SD_FLUSH_CH_ADSB);
    }
}

// Close the log permanently (for clean shutdown/unmount).
void sd_log_close(void) {
    if (!sd_ch.initialized) return;
    sd_log_flush();
    sd_log_ch_close(&sd_ch);
}

// Print current SD log status.
void sd_log_print_status(void) {
    if (sd_ch.initialized && sd_ch.filename[0]) {
        printf("I (%lu) CLASS: SD log: %s\n",
            (unsigned long)(esp_timer_get_time() / 1000), sd_ch.filename);
    }
}

// Check if ADS-B SD logging is active (log file open and header written).
bool sd_log_is_active(void) {
    return sd_ch.initialized;
}

// Close current log and create a fresh one with a new boot-numbered filename.
// Called after MSC mode exit to start a clean log file.
void sd_log_create_new(void) {
    if (sd_ch.initialized) sd_log_close();
    sd_log_create();
}

// Rename boot-numbered log to UTC-timestamped name.
// Called once from GPS task when the system clock is first set.
void sd_log_rename_with_time(void) {
    if (!sd_ch.initialized) return;
    if (strstr(sd_ch.filename, "adsb_boot") == NULL) return;
    if (!time_manager_is_trusted_fn()) return;  // wait for GPS/NTP

    ESP_LOGI(TAG, "SD rename starting [DMA free=%u largest=%u, internal=%u]",
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    struct tm timeinfo;
    time_t now;
    time(&now);
    gmtime_r(&now, &timeinfo);

    char new_filename[64];
    strftime(new_filename, sizeof(new_filename),
        "/sdcard/logs/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);

    if (sd_log_ch_rename(&sd_ch, new_filename, sd_log_buf, sd_log_buf_pos)) {
        sd_log_buf_pos = 0;  // buffer was flushed by rename
    } else {
        // Rename failed — channel is now uninitialized.  Create a fresh
        // file (sd_log_create picks a new timestamp name + writes header).
        // Buffer was already flushed by rename before the failure.
        sd_log_buf_pos = 0;
        sd_log_create();
    }
}

// Append a formatted timestamp to the PSRAM buffer.
static void sd_log_buf_timestamp(void) {
    if (!sd_log_buf) return;
    int avail = SD_LOG_BUFSIZE - sd_log_buf_pos;
    if (avail < 40) return;  // need room for timestamp

    int n;
    if (time_manager_is_trusted_fn()) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        struct tm tm_info;
        gmtime_r(&tv.tv_sec, &tm_info);
        n = snprintf(sd_log_buf + sd_log_buf_pos, avail,
            "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
            tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
            tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
            (long)(tv.tv_usec / 1000));
    } else {
        int64_t boot_ms = esp_timer_get_time() / 1000;
        n = snprintf(sd_log_buf + sd_log_buf_pos, avail,
            "boot+%lld.%03lld",
            (long long)(boot_ms / 1000), (long long)(boot_ms % 1000));
    }
    if (n > 0) sd_log_buf_pos += n;
}

static void sd_log_aircraft(aircraft_t *ac, const struct mode_s_msg *mm) {
    // Don't write to SD while USB MSC owns the card
    extern bool sd_msc_is_active_fn(void);
    if (sd_msc_is_active_fn()) return;

    // Allocate buffer once; init channel on first call
    if (!sd_log_buf) {
        sd_log_buf = heap_caps_malloc(SD_LOG_BUFSIZE, MALLOC_CAP_SPIRAM);
        if (!sd_log_buf) return;
        sd_log_buf_pos = 0;
        sd_log_ch_init(&sd_ch, TAG);
    }

    // Lazy create: retry if the log hasn't been created yet
    if (!sd_ch.initialized) {
        // If the FAT is structurally corrupt, stop retrying entirely.
        // User must reformat the card and run 'mount'.
        if (sd_log_bus_failed()) return;

        static int64_t last_retry = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_retry < 5000000LL) return;  // 5s throttle
        last_retry = now_us;

        extern bool sd_is_mounted(void);

        // 1. If we have a previous filename and card is mounted, try reopen
        if (sd_ch.filename[0] && sd_is_mounted()) {
            sd_log_ch_reopen(&sd_ch);
            if (sd_ch.initialized && sd_log_buf_pos > 0) {
                sd_log_flush();  // flush any buffered data from before failure
            }
        }

        // 2. Card mounted but reopen failed (or no previous file) → new file
        if (!sd_ch.initialized && sd_is_mounted()) {
            sd_log_create();
            if (sd_ch.initialized && sd_log_buf_pos > 0) {
                sd_log_flush();  // flush old buffer to new file
            }
        }

        // 3. Card not mounted → try recovery, then reopen or create
        if (!sd_ch.initialized) {
            if (sd_log_try_recovery(TAG)) {
                // Recovery succeeded — try reopen original file
                if (sd_ch.filename[0]) {
                    sd_log_ch_reopen(&sd_ch);
                }
                // If reopen failed, create new file
                if (!sd_ch.initialized) {
                    sd_log_create();
                }
                if (sd_ch.initialized && sd_log_buf_pos > 0) {
                    sd_log_flush();
                }
            }
        }

        // 4. Card is mounted but we still can't create files → FAT is
        //    structurally corrupt (zombie card).  Stop retrying to avoid
        //    an infinite 5s loop of failed fopen() calls.
        if (!sd_ch.initialized && sd_is_mounted()) {
            sd_log_bus_fail("Card mounted but cannot create files "
                            "— FAT corrupted, reformat card and run 'mount'");
        }

        if (!sd_ch.initialized) return;
    }

    // Check buffer space — flush if getting full (leave 512B headroom)
    if (sd_log_buf_pos > SD_LOG_BUFSIZE - 512) {
        sd_log_flush();
    }

    receiver_pos_t rx = adsb_get_receiver_pos();
    double range_km = 0, bearing = 0;
    if (rx.fix_valid && ac->has_position)
        haversine(rx.lat, rx.lon, ac->lat, ac->lon, &range_km, &bearing);

    // Write to PSRAM buffer (not to disk)
    sd_log_buf_timestamp();

    int avail = SD_LOG_BUFSIZE - sd_log_buf_pos;
    if (avail < 256) return;  // safety margin

    // Raw hex message
    int n = snprintf(sd_log_buf + sd_log_buf_pos, avail, ",");
    if (n > 0) sd_log_buf_pos += n;

    if (mm) {
        int msglen = mm->msgbits / 8;
        for (int i = 0; i < msglen && sd_log_buf_pos < SD_LOG_BUFSIZE - 4; i++) {
            n = snprintf(sd_log_buf + sd_log_buf_pos, 3, "%02X", mm->msg[i]);
            if (n > 0) sd_log_buf_pos += n;
        }
    }

    avail = SD_LOG_BUFSIZE - sd_log_buf_pos;
    n = snprintf(sd_log_buf + sd_log_buf_pos, avail,
        ",%06lX,%s,%d,%d,%d,%d,%.5f,%.5f,%04d,%.5f,%.5f,%.1f,%.0f,%d,%.1f\n",
        (unsigned long)ac->icao,
        ac->callsign[0] ? ac->callsign : "",
        ac->altitude,
        ac->speed,
        ac->heading,
        ac->vert_rate,
        ac->lat,
        ac->lon,
        ac->squawk,
        rx.fix_valid ? rx.lat : 0.0,
        rx.fix_valid ? rx.lon : 0.0,
        range_km,
        bearing,
        rx.sats,
        rx.hdop
    );
    if (n > 0) sd_log_buf_pos += n;

    // Periodic flush:
    //   • Buffer nearly full → immediate (safety)
    //   • ≥1s since last flush AND ≥512 bytes → flush (one full sector min)
    //   • ≥5s since last flush → flush regardless of size (low-traffic fallback)
    int64_t sync_now = esp_timer_get_time();
    int64_t since_self = sync_now - sd_log_last_sync;
    bool buf_critical = sd_log_buf_pos > SD_LOG_BUFSIZE - 512;
    bool has_block = sd_log_buf_pos >= SD_SYNC_MIN_BYTES;

    bool should_flush = false;
    if (buf_critical) {
        should_flush = true;
    } else if (since_self >= SD_SYNC_MAX_US && sd_log_buf_pos > 0) {
        should_flush = true;                                          // 5s hard max
    } else if (since_self >= SD_SYNC_INTERVAL_US && has_block) {
        should_flush = true;                                          // 1s + ≥512B
    }

    if (should_flush) {
        sd_log_flush();
    }
}
// ============================================================

// ============================================================
// LVGL display
// ============================================================

void adsb_update_display(lv_obj_t *table) {
    // LVGL table cell updates disabled — lv_table_set_cell_value calls realloc
    // which crashes on heap metadata corrupted by USB DMA.  Table shows headers
    // only.  Aircraft data is logged to SD card via sd_log_aircraft().
    (void)table;
    return;
}

// ============================================================
// on_msg
// ============================================================

void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    // mm is already fully decoded by mode_s_detect() → mode_s_decode()
    // before invoking this callback.  Do NOT re-decode: mode_s_decode()
    // does memset(mm,0,...) which zeroes mm->msg, and since the third
    // arg (msg) points into the same struct, the copy-back reads zeros.

    uint32_t icao = (uint32_t)(mm->aa1 << 16 | mm->aa2 << 8 | mm->aa3);
    if (icao == 0) return;

    if (!aircraft_mutex) return;
    xSemaphoreTake(aircraft_mutex, portMAX_DELAY);
    aircraft_t *ac = aircraft_get(icao);
    ac->last_seen = esp_timer_get_time();
    ac->msg_count++;

    // Emit aircraft database info on first message for a new ICAO.
    // Separate log line so we don't change the ADS-B data format.
    // Webapp and firmware scope cache this for detail display.
    if (ac->msg_count == 1 && aircraft_db_ready()) {
        aircraft_db_entry_t db;
        if (aircraft_db_lookup(icao, &db)) {
            char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
            serial_console_print("%sICAO: %06lX|%s|%s|%s|%s|%s|%s|%c\n",
                _ts, (unsigned long)icao,
                db.reg, db.typecode, db.mfr, db.model, db.owner, db.op_icao,
                db.ac_class ? db.ac_class : '?');
        }
    }

    // --- Update aircraft state from this message ---

    // Callsign (DF17, ME 1-4)
    if (mm->msgtype == 17 && mm->metype >= 1 && mm->metype <= 4) {
        strncpy(ac->callsign, mm->flight, 8);
        ac->callsign[8] = '\0';
        for (int i = 7; i >= 0 && ac->callsign[i] == ' '; i--)
            ac->callsign[i] = '\0';
    }

    // Altitude — only from message types that carry it:
    //   DF0/4/16/20: 13-bit AC field (decode_ac13_field)
    //   DF17 TC 9-18: 12-bit altitude in airborne position (decode_ac12_field)
    if (mm->msgtype == 0 || mm->msgtype == 4 ||
        mm->msgtype == 16 || mm->msgtype == 20) {
        if (mm->altitude) ac->altitude = mm->altitude;
    } else if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18) {
        if (mm->altitude) ac->altitude = mm->altitude;
    }

    // CPR position (DF17, TC 9-18)
    if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18) {
        cpr_cache_t *c = cpr_get(icao);
        int64_t now = esp_timer_get_time();
        if (mm->fflag) {
            c->odd_lat   = mm->raw_latitude;
            c->odd_lon   = mm->raw_longitude;
            c->odd_ts    = now;
            c->odd_valid = 1;
        } else {
            c->even_lat   = mm->raw_latitude;
            c->even_lon   = mm->raw_longitude;
            c->even_ts    = now;
            c->even_valid = 1;
        }

        int decoded = 0;

        // ── CPR decode strategy with position jump detection:
        //
        //    Every candidate position (local or global) is jump-checked against
        //    the last known good position (good_lat/good_lon). Implausible jumps
        //    trigger suspect mode, which suppresses local decode (poisoned reference)
        //    and waits for a clean global decode to recover.
        //
        //    Jump threshold: max(10nm, elapsed_seconds × 1.0 nm/s) — ~3600 kt,
        //    faster than any transponder-equipped aircraft including hypersonic.
        //    ──

        #define CPR_TIGHT_AGE_US  2000000LL  // 2 seconds — tight global is trustworthy

        int64_t frame_age = c->even_ts - c->odd_ts;
        if (frame_age < 0) frame_age = -frame_age;

        // 1. Local decode: primary path (skip if suspect — reference is poisoned)
        if (ac->has_position && !ac->position_suspect) {
            double new_lat, new_lon;
            if (cpr_decode_local(ac->lat, ac->lon,
                                 mm->raw_latitude, mm->raw_longitude,
                                 mm->fflag, &new_lat, &new_lon)) {
                if (cpr_jump_check(ac, new_lat, new_lon, now)) {
                    // Good local decode — accept
                    ac->lat = new_lat;
                    ac->lon = new_lon;
                    c->lat = new_lat;
                    c->lon = new_lon;
                    ac->good_lat = new_lat;
                    ac->good_lon = new_lon;
                    ac->good_pos_ts = now;
                    ac->suspect_count = 0;
                    decoded = 1;
                } else {
                    // Local jumped — reference is poisoned, enter suspect mode
                    ac->position_suspect = 1;
                    ac->suspect_count = 1;
                    ac->suspect_agree = 0;
                    ac->suspect_lat = 0;
                    ac->suspect_lon = 0;
                    serial_console_print("CPR: %06lX position suspect — local jumped, awaiting global recovery\n",
                                         ac->icao);
                    // Fall through to global decode for immediate recovery attempt
                }
            }
        }

        // 2. Global decode
        if (c->even_valid && c->odd_valid && frame_age <= CPR_MAX_AGE_US) {
            if (cpr_decode(c)) {
                double gLat = c->lat, gLon = c->lon;

                if (!ac->has_position) {
                    // First contact — no good_pos, jump check auto-passes
                    ac->lat = gLat;
                    ac->lon = gLon;
                    ac->has_position = 1;
                    ac->good_lat = gLat;
                    ac->good_lon = gLon;
                    ac->good_pos_ts = now;
                    decoded = 1;

                } else if (ac->position_suspect) {
                    // Recovery mode — two paths to accept:
                    //   A. Global passes jump check against good_pos (good_pos was right)
                    //   B. N consecutive globals agree with each other (good_pos was wrong)

                    int recovered = 0;
                    int via_consensus = 0;

                    // Path A: matches good_pos — immediate recovery
                    if (cpr_jump_check(ac, gLat, gLon, now)) {
                        recovered = 1;
                    }

                    // Path B: consensus — globals agreeing with each other
                    if (!recovered) {
                        if (ac->suspect_agree > 0) {
                            // Check against previous candidate
                            double dlat = gLat - ac->suspect_lat;
                            double dlon = (gLon - ac->suspect_lon) * cos(ac->suspect_lat * M_PI / 180.0);
                            double dist_nm = sqrt(dlat * dlat + dlon * dlon) * 60.0;
                            if (dist_nm <= CPR_CONSENSUS_NM) {
                                ac->suspect_agree++;
                                ac->suspect_lat = gLat;  // track latest in cluster
                                ac->suspect_lon = gLon;
                                if (ac->suspect_agree >= CPR_CONSENSUS_COUNT) {
                                    recovered = 1;
                                    via_consensus = 1;
                                }
                            } else {
                                // Doesn't match previous candidate — restart consensus
                                ac->suspect_agree = 1;
                                ac->suspect_lat = gLat;
                                ac->suspect_lon = gLon;
                            }
                        } else {
                            // First candidate in recovery — seed consensus
                            ac->suspect_agree = 1;
                            ac->suspect_lat = gLat;
                            ac->suspect_lon = gLon;
                        }
                    }

                    if (recovered) {
                        ac->lat = gLat;
                        ac->lon = gLon;
                        c->lat = gLat;
                        c->lon = gLon;
                        ac->position_suspect = 0;
                        ac->suspect_count = 0;
                        ac->suspect_agree = 0;
                        ac->good_lat = gLat;
                        ac->good_lon = gLon;
                        ac->good_pos_ts = now;
                        decoded = 1;
                        if (via_consensus)
                            serial_console_print("CPR: %06lX recovered via consensus (%d agreeing globals)\n",
                                                 ac->icao, CPR_CONSENSUS_COUNT);
                        else
                            serial_console_print("CPR: %06lX recovered via global decode\n", ac->icao);
                    } else {
                        ac->suspect_count++;
                        if (ac->suspect_count >= CPR_SUSPECT_GIVE_UP) {
                            ac->has_position = 0;
                            ac->position_suspect = 0;
                            ac->suspect_count = 0;
                            ac->suspect_agree = 0;
                            serial_console_print("CPR: %06lX gave up recovery — resetting position\n",
                                                 ac->icao);
                        }
                    }

                } else if (!decoded) {
                    // Local failed (50nm check) — try global as re-seed
                    if (cpr_jump_check(ac, gLat, gLon, now)) {
                        ac->lat = gLat;
                        ac->lon = gLon;
                        ac->good_lat = gLat;
                        ac->good_lon = gLon;
                        ac->good_pos_ts = now;
                        decoded = 1;
                    } else {
                        // Global re-seed also implausible — enter suspect
                        ac->position_suspect = 1;
                        ac->suspect_count = 1;
                        ac->suspect_agree = 0;
                        ac->suspect_lat = 0;
                        ac->suspect_lon = 0;
                        serial_console_print("CPR: %06lX position suspect — global re-seed jumped\n",
                                             ac->icao);
                    }

                } else if (frame_age <= CPR_TIGHT_AGE_US) {
                    // Keyframe correction: local succeeded, tight global available
                    if (cpr_jump_check(ac, gLat, gLon, now)) {
                        double dlat = ac->lat - gLat;
                        double dlon = (ac->lon - gLon) * cos(ac->lat * M_PI / 180.0);
                        double dist_nm = sqrt(dlat * dlat + dlon * dlon) * 60.0;
                        if (dist_nm > 2.0) {
                            // Local reference has drifted — reseat from global
                            ac->lat = gLat;
                            ac->lon = gLon;
                            ac->good_lat = gLat;
                            ac->good_lon = gLon;
                            ac->good_pos_ts = now;
                        }
                    } else {
                        // Tight global disagrees with good_pos — good_pos may be wrong.
                        // Both local and tight global agree (decoded=1 + tight pair),
                        // but both are far from good_pos. Enter suspect, seed consensus.
                        ac->position_suspect = 1;
                        ac->suspect_count = 1;
                        ac->suspect_agree = 1;
                        ac->suspect_lat = gLat;
                        ac->suspect_lon = gLon;
                        serial_console_print("CPR: %06lX position suspect — tight global disagrees with reference\n",
                                             ac->icao);
                    }
                    // Loose global (2–10s) with local already decoded: skip.
                }
            }
        }
        if (decoded) s_gain_win_pos++;  // track for adaptive gain throughput metric
    }

    // Velocity (DF17, TC 19, subtypes 1-4)
    if (mm->msgtype == 17 && mm->metype == 19) {
        if (mm->mesub == 1 || mm->mesub == 2) {
            ac->speed = mm->velocity;
            // Heading is computed from EW/NS velocity components in mode_s_decode
            // and is always valid when velocity > 0 (heading_is_valid is only
            // meaningful for subtypes 3/4 which carry airborne magnetic heading).
            if (mm->velocity)
                ac->heading = mm->heading;
            // vert_rate raw=0 means "not available" per DO-260B
            if (mm->vert_rate) {
                ac->vert_rate = mm->vert_rate_sign
                              ? -((mm->vert_rate - 1) * 64)
                              :  ((mm->vert_rate - 1) * 64);
            }
        } else if (mm->mesub == 3 || mm->mesub == 4) {
            if (mm->heading_is_valid)
                ac->heading = mm->heading;
        }
    }

    // Squawk (DF5, DF21)
    if (mm->msgtype == 5 || mm->msgtype == 21) {
        ac->squawk = mm->identity;
    }

    // --- Print one-line summary with full known state ---
    // Routes through serial_console_print() so lines are buffered in PSRAM
    // during CMD mode and replayed when returning to LOG mode.
    {
    char line[384];
    int pos = 0, avail = sizeof(line);

    // Timestamp — shared format with GNSS and Meshy log lines
    { int n = log_format_timestamp(line + pos, avail);
      if (n > 0) { pos += n; avail -= n; } }

    // ICAO + callsign
    { int n = snprintf(line + pos, avail, "%06lX", (unsigned long)icao);
      if (n > 0) { pos += n; avail -= n; } }
    if (ac->callsign[0]) {
        int n = snprintf(line + pos, avail, " %-8s", ac->callsign);
        if (n > 0) { pos += n; avail -= n; }
    }

    // Altitude + velocity
    if (ac->altitude) {
        int n = snprintf(line + pos, avail, " %5dft", ac->altitude);
        if (n > 0) { pos += n; avail -= n; }
    }
    if (ac->speed) {
        int n = snprintf(line + pos, avail, " %3dkt %03d°", ac->speed, ac->heading);
        if (n > 0) { pos += n; avail -= n; }
    }
    if (ac->vert_rate) {
        int n = snprintf(line + pos, avail, " %+dfpm", ac->vert_rate);
        if (n > 0) { pos += n; avail -= n; }
    }

    // Position + distance from receiver
    if (ac->has_position) {
        int n = snprintf(line + pos, avail, " (%.4f%c,%.4f%c)",
            fabs(ac->lat), ac->lat >= 0 ? 'N' : 'S',
            fabs(ac->lon), ac->lon >= 0 ? 'E' : 'W');
        if (n > 0) { pos += n; avail -= n; }
        receiver_pos_t rx = adsb_get_receiver_pos();
        if (rx.fix_valid) {
            double dist_km, brg;
            haversine(rx.lat, rx.lon, ac->lat, ac->lon, &dist_km, &brg);
            double dist_nm = dist_km * 0.539957;
            n = snprintf(line + pos, avail, " %.1fnm@%03.0f°", dist_nm, brg);
            if (n > 0) { pos += n; avail -= n; }
        }
    }

    // Squawk
    if (ac->squawk) {
        int n = snprintf(line + pos, avail, " SQK:%04d", ac->squawk);
        if (n > 0) { pos += n; avail -= n; }
    }

    // Raw hex (compact, at end)
    { int n = snprintf(line + pos, avail, " [");
      if (n > 0) { pos += n; avail -= n; } }
    int msglen = mm->msgbits / 8;
    for (int i = 0; i < msglen && avail > 3; i++) {
        int n = snprintf(line + pos, avail, "%02X", mm->msg[i]);
        if (n > 0) { pos += n; avail -= n; }
    }
    if (avail > 2) { line[pos++] = ']'; line[pos++] = '\n'; line[pos] = '\0'; }

    serial_console_print("%s", line);
    }

    sd_log_aircraft(ac, mm);

    // Queue aircraft update for MQTT feeder (returns immediately if not connected)
    {
        mqtt_aircraft_t mqac = {0};
        snprintf(mqac.icao, sizeof(mqac.icao), "%06lX", (unsigned long)icao);
        if (ac->callsign[0])
            strncpy(mqac.callsign, ac->callsign, sizeof(mqac.callsign) - 1);
        mqac.lat = ac->lat;
        mqac.lon = ac->lon;
        mqac.alt_ft = ac->altitude;
        mqac.speed_kt = (int16_t)ac->speed;
        mqac.heading_deg = (int16_t)ac->heading;
        mqac.vert_rate_fpm = (int16_t)ac->vert_rate;
        mqac.has_pos = ac->has_position ? true : false;
        mqac.ts_us = esp_timer_get_time();
        // Compute distance/bearing from receiver
        receiver_pos_t rx = adsb_get_receiver_pos();
        if (rx.fix_valid && ac->has_position) {
            double dist_km, brg;
            haversine(rx.lat, rx.lon, ac->lat, ac->lon, &dist_km, &brg);
            mqac.dist_nm = (float)(dist_km * 0.539957);
            mqac.bearing_deg = (int16_t)brg;
        }
        mqtt_feeder_update_aircraft(&mqac);
    }

    // Update message statistics
    s_total_messages++;
    s_msg_count_window++;
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed = now_us - s_msg_window_start;
    if (elapsed >= 1000000LL) {  // 1 second window
        s_msg_rate = (float)s_msg_count_window * 1000000.0f / (float)elapsed;
        s_msg_count_window = 0;
        s_msg_window_start = now_us;
    }

    xSemaphoreGive(aircraft_mutex);
}

// ============================================================
// demodulate — uses pre-allocated magnitude buffer
// ============================================================

void demodulate(uint8_t *source, int length)
{
    if (!s_mag_buf) return;
    mode_s_compute_magnitude_vector(source, s_mag_buf, length);
    mode_s_detect(&state, s_mag_buf, length / 2, on_msg);
}

// ============================================================
// ADSB reader task — runs the blocking RTL-SDR read loop
// in its own FreeRTOS task instead of inside the USB callback.
// ============================================================

static void adsb_reader_task(void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    int r, n_read = 0;

    uint8_t dev_addr = s_new_dev_addr;
    ESP_LOGI(TAG, "ADSB reader task started for device %d", dev_addr);
    // Open and configure RTL-SDR
    r = rtlsdr_open(&rtldev, dev_addr,
        driver_obj->mux_protected.device[dev_addr].client_hdl);
    if (r < 0) {
        ESP_LOGE(TAG, "rtlsdr_open failed: %d", r);
        goto done;
    }

    /* Log dongle identification for debugging */
    {
        char model_str[64];
        int force_bt = 0;
        rtlsdr_get_dongle_info(rtldev, model_str, sizeof(model_str), NULL, &force_bt);
        ESP_LOGI(TAG, "RTL-SDR: %s%s", model_str, force_bt ? ", bias-T forced ON" : "");
    }

    r = rtlsdr_set_center_freq(rtldev, 1090000000);
    if (r < 0) fprintf(stderr, "WARNING: Failed to set center freq.\n");
    else       fprintf(stderr, "Tuned to %u Hz.\n", 1090000000);

    r = rtlsdr_set_sample_rate(rtldev, 2000000);
    if (r < 0) fprintf(stderr, "WARNING: Failed to set sample rate.\n");
    else       fprintf(stderr, "Sampling at %u S/s.\n", 2000000);

    r = rtlsdr_set_tuner_gain_mode(rtldev, 1);  // 1 = manual gain
    if (r != 0) fprintf(stderr, "WARNING: Failed to set tuner gain mode.\n");

    // Initialize gain from settings
    if (g_settings.adsb_gain_mode == 1) {
        // Manual: use saved value
        s_gain_auto = false;
        s_gain_idx = gain_find_idx(g_settings.adsb_gain_tenths);
    } else {
        // Auto: start at max, let adaptive algorithm adjust
        s_gain_auto = true;
        s_gain_idx = R820T_GAIN_COUNT - 1;
    }
    gain_apply();
    fprintf(stderr, "Tuner gain set to %.1f dB (%s).\n",
            R820T_GAINS[s_gain_idx] / 10.0, s_gain_auto ? "adaptive" : "manual");

    // Apply bias-T setting from NVS (default: off)
    r = rtlsdr_set_bias_tee(rtldev, g_settings.adsb_bias_tee ? 1 : 0);
    if (r == 0) ESP_LOGI(TAG, "Bias-T: %s", g_settings.adsb_bias_tee ? "ON" : "OFF");

    r = rtlsdr_reset_buffer(rtldev);
    if (r < 0) fprintf(stderr, "WARNING: Failed to reset buffers.\n");

    vTaskDelay(pdMS_TO_TICKS(100));
    alloc_adsb_transfer();

    // Only mark connected if the bulk transfer buffer is usable.
    // Without it, reads fail on every call — CIT and status bar
    // should reflect the error, not show a green checkmark.
    if (adsb_transfer_ready()) {
        s_rtlsdr_connected = true;
        s_rtlsdr_error = false;
    } else {
        s_rtlsdr_connected = false;
        s_rtlsdr_error = true;  // device enumerated but buffer alloc failed
        ESP_LOGE(TAG, "RTL-SDR init completed but transfer buffer unavailable — reads will fail");
    }

    ESP_LOGI(TAG, "[APP] Free memory: %ld bytes", esp_get_free_heap_size());

    mode_s_init(&state);
    ESP_LOGI(TAG, "mode_s_init done, stack HWM: %u free, heap: %s",
             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)),
             heap_caps_check_integrity_all(true) ? "OK" : "CORRUPT");

    // Allocate SD log buffer and initialize channel before creating the log
    // file.  sd_log_aircraft() has a lazy-init path, but it re-inits the
    // channel (memset) which would zero out an already-open file handle if
    // sd_log_create() ran first.  Do the allocation here so both paths agree.
    if (!sd_log_buf) {
        sd_log_buf = heap_caps_malloc(SD_LOG_BUFSIZE, MALLOC_CAP_SPIRAM);
        if (sd_log_buf) {
            sd_log_buf_pos = 0;
            sd_log_ch_init(&sd_ch, TAG);
        }
    }
    sd_log_create();

    s_msg_window_start = esp_timer_get_time();

    // Allocate read buffers and magnitude buffer once — in PSRAM to
    // preserve internal RAM for SDMMC DMA, LVGL, and wallpaper decoding.
    uint8_t *buffer = heap_caps_malloc(DEFAULT_BUF_LENGTH * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    if (buffer == NULL) { ESP_LOGE(TAG, "buffer malloc failed"); goto done; }

    uint8_t *tbuffer = heap_caps_malloc(MAX_PACKET_SIZE * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    if (tbuffer == NULL) {
        ESP_LOGE(TAG, "tbuffer malloc failed");
        free(buffer);
        goto done;
    }

    s_mag_buf = heap_caps_malloc((DEFAULT_BUF_LENGTH / 2) * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (s_mag_buf == NULL) {
        ESP_LOGE(TAG, "mag buffer malloc failed");
        free(tbuffer);
        free(buffer);
        goto done;
    }
    // Main read loop — runs until stop flag is set (e.g. device removed)
    int consecutive_errors = 0;
    s_gain_last_eval = esp_timer_get_time();

    while (!s_adsb_reader_stop) {
        // Apply deferred gain changes from external callers (serial console)
        // This ensures rtlsdr_set_tuner_gain() is only called from the USB task.
        if (s_gain_pending) {
            gain_apply();
            s_gain_pending = false;
        }

        int read_ok = 1;
        for (int i = 0; i < DEFAULT_BUF_LENGTH; i += MAX_PACKET_SIZE) {
            r = rtlsdr_read_sync(rtldev, tbuffer, MAX_PACKET_SIZE, &n_read);
            if (r < 0) {
                fprintf(stderr, "WARNING: sync read failed. %d\n", r);
                read_ok = 0;
                break;
            }
            if ((uint32_t)n_read < MAX_PACKET_SIZE) {
                fprintf(stderr, "Short read\n");
                read_ok = 0;
                break;
            }
            memcpy(&buffer[i], tbuffer, MAX_PACKET_SIZE);
        }
        if (read_ok && n_read > 0) {
            demodulate(buffer, DEFAULT_BUF_LENGTH);
            consecutive_errors = 0;  // reset on successful read

            // One-shot health check after first successful read
            {
                static bool first_read_logged = false;
                if (!first_read_logged) {
                    first_read_logged = true;
                    ESP_LOGI(TAG, "First read OK, stack HWM: %u free, heap: %s",
                             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)),
                             heap_caps_check_integrity_all(true) ? "OK" : "CORRUPT");
                }
            }

            // Adaptive gain evaluation — phase-dependent timing
            if (s_gain_auto) {
                int64_t now_gain = esp_timer_get_time();
                int64_t elapsed = now_gain - s_gain_last_eval;
                uint32_t msgs = state.stat_crc_ok + state.stat_crc_fail;
                bool should_eval = false;

                // Warmup: let decoder stabilize before first evaluation
                if (s_gain_warmup && elapsed < GAIN_WARMUP_US) {
                    // Still warming up — skip
                } else {
                    if (s_gain_warmup) {
                        // Warmup just ended — flush accumulated counters so first real window starts clean
                        s_gain_warmup = false;
                        state.stat_crc_ok = 0;
                        state.stat_crc_fail = 0;
                        state.stat_preambles = 0;
                        state.stat_signal_sum = 0;
                        state.stat_delta_sum = 0;
                        s_gain_win_pos = 0;
                        s_gain_last_eval = now_gain;
                        ESP_LOGI("GAIN", "Warmup complete — starting adaptive gain");
                    } else if (s_gain_phase == 1) {
                        // Phase 1: fast — every 10s or 200 messages
                        if (elapsed >= GAIN_P1_PERIOD_US || msgs >= GAIN_P1_MSG_COUNT)
                            should_eval = true;
                    } else {
                        // Phase 2: steady — every 60s
                        if (elapsed >= GAIN_P2_PERIOD_US)
                            should_eval = true;
                    }

                    if (should_eval) {
                        gain_evaluate(&state);
                        s_gain_last_eval = now_gain;
                    }
                }
            }

            // Manual-mode heartbeat: log stats every 60s for webapp visibility
            if (!s_gain_auto) {
                static int64_t s_manual_hb_last = 0;
                static uint32_t s_manual_hb_ok = 0;
                static uint32_t s_manual_hb_fail = 0;
                static uint32_t s_manual_hb_pos = 0;
                static uint64_t s_manual_hb_signal = 0;
                static uint64_t s_manual_hb_delta = 0;
                int64_t now_hb = esp_timer_get_time();
                if (s_manual_hb_last == 0) s_manual_hb_last = now_hb;  // first call
                if (now_hb - s_manual_hb_last >= GAIN_P2_PERIOD_US) {
                    float elapsed_s = (now_hb - s_manual_hb_last) / 1e6f;
                    uint32_t d_ok = state.stat_crc_ok - s_manual_hb_ok;
                    uint32_t d_fail = state.stat_crc_fail - s_manual_hb_fail;
                    uint32_t d_pos = s_gain_win_pos - s_manual_hb_pos;
                    uint64_t d_signal = state.stat_signal_sum - s_manual_hb_signal;
                    uint64_t d_delta_sum = state.stat_delta_sum - s_manual_hb_delta;
                    float err = (d_ok + d_fail > 0) ? (float)d_fail / (d_ok + d_fail) : 0;
                    float pos_rate = (elapsed_s > 0) ? d_pos / elapsed_s : 0;
                    uint32_t hb_avg_sig = d_ok > 0 ? (uint32_t)(d_signal / d_ok) : 0;
                    uint32_t hb_avg_del = d_ok > 0 ? (uint32_t)(d_delta_sum / d_ok) : 0;
                    s_manual_hb_last = now_hb;
                    s_manual_hb_ok = state.stat_crc_ok;
                    s_manual_hb_fail = state.stat_crc_fail;
                    s_manual_hb_pos = s_gain_win_pos;
                    s_manual_hb_signal = state.stat_signal_sum;
                    s_manual_hb_delta = state.stat_delta_sum;
                    char _ts[32]; log_format_timestamp(_ts, sizeof(_ts));
                    serial_console_print("%sGAIN: [Manual] %.1f dB, err=%.0f%%, pos=%.1f/s, range=%.0fnm, sig=%lu delta=%lu\n",
                                         _ts, R820T_GAINS[s_gain_idx] / 10.0,
                                         err * 100, pos_rate, s_max_range_nm,
                                         (unsigned long)hb_avg_sig, (unsigned long)hb_avg_del);
                }
            }
        }
        if (!read_ok) {
            consecutive_errors++;
            if (consecutive_errors >= 10) {
                ESP_LOGE(TAG, "Too many consecutive read errors (%d) — assuming device disconnected", consecutive_errors);
                break;
            }
            // Back off briefly on read errors before retrying
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    free(s_mag_buf);
    s_mag_buf = NULL;
    free(tbuffer);
    free(buffer);

done:
    s_rtlsdr_connected = false;
    s_rtlsdr_error = false;  // device gone, no longer an error — just disconnected
    // Close RTL-SDR device (releases USB interface claims)
    if (rtldev != NULL) {
        rtlsdr_close(rtldev);
        rtldev = NULL;
    }
    // Keep the bulk transfer buffer allocated — it was pre-allocated early
    // in boot when internal RAM was contiguous. Freeing it risks never
    // getting it back due to heap fragmentation.  alloc_adsb_transfer()
    // will see it's already allocated and skip on reconnect.
    sd_log_close();
    // Release USB event pumping back to class_driver_task
    s_reader_owns_events = false;
    ESP_LOGI(TAG, "ADSB reader task exiting");
    s_adsb_reader_task_hdl = NULL;
    vTaskDelete(NULL);
}

// ============================================================
// USB client event callback — now non-blocking
// ============================================================

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;

    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr = event_msg->new_dev.address;
        xSemaphoreGive(driver_obj->constant.mux_lock);

        // Launch the reader task — don't block in the callback.
        // Set the event ownership flag BEFORE creating the task so that
        // class_driver_task sees it immediately when this callback returns,
        // avoiding a window where both tasks pump USB events concurrently.
        s_new_dev_addr = event_msg->new_dev.address;
        s_adsb_reader_stop = false;
        if (s_adsb_reader_task_hdl == NULL) {
            s_reader_owns_events = true;
            xTaskCreate(adsb_reader_task, "adsb_reader", ADSB_READER_TASK_STACK,
                        (void *)driver_obj, ADSB_READER_TASK_PRIO,
                        &s_adsb_reader_task_hdl);
        }
        break;

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        // Signal the reader task to stop
        s_adsb_reader_stop = true;

        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
            if (driver_obj->mux_protected.device[i].dev_hdl == event_msg->dev_gone.dev_hdl) {
                driver_obj->mux_protected.device[i].actions = ACTION_CLOSE_DEV;
                driver_obj->mux_protected.flags.unhandled_devices = 1;
            }
        }
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;

    default:
        ESP_LOGW(TAG, "Unknown USB client event: %d", event_msg->event);
        break;
    }
}

// ============================================================
// Action handlers
// ============================================================

static void action_open_dev(usb_device_t *device_obj)
{
    assert(device_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", device_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(device_obj->client_hdl, device_obj->dev_addr, &device_obj->dev_hdl));
    device_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG, "\t%s speed", (char *[]){"Low", "Full", "High"}[dev_info.speed]);
    ESP_LOGI(TAG, "\tParent info:");
    if (dev_info.parent.dev_hdl) {
        usb_device_info_t parent_dev_info;
        ESP_ERROR_CHECK(usb_host_device_info(dev_info.parent.dev_hdl, &parent_dev_info));
        ESP_LOGI(TAG, "\t\tBus addr: %d", parent_dev_info.dev_addr);
        ESP_LOGI(TAG, "\t\tPort: %d", dev_info.parent.port_num);
    } else {
        ESP_LOGI(TAG, "\t\tPort: ROOT");
    }
    ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    device_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(device_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    device_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);
    device_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer) {
        ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product) {
        ESP_LOGI(TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num) {
        ESP_LOGI(TAG, "Getting Serial Number string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_serial_num);
    }
}

static void action_close_dev(usb_device_t *device_obj)
{
    if (device_obj->dev_hdl == NULL) return;  // already closed (e.g. by rtlsdr_close)
    ESP_ERROR_CHECK(usb_host_device_close(device_obj->client_hdl, device_obj->dev_hdl));
    device_obj->dev_hdl = NULL;
    device_obj->dev_addr = 0;
}

static void class_driver_device_handle(usb_device_t *device_obj)
{
    uint8_t actions = device_obj->actions;
    device_obj->actions = 0;
    while (actions) {
        if (actions & ACTION_OPEN_DEV)        action_open_dev(device_obj);
        if (actions & ACTION_GET_DEV_INFO)    action_get_info(device_obj);
        if (actions & ACTION_GET_DEV_DESC)    action_get_dev_desc(device_obj);
        if (actions & ACTION_GET_CONFIG_DESC) action_get_config_desc(device_obj);
        if (actions & ACTION_GET_STR_DESC)    action_get_str_desc(device_obj);
        if (actions & ACTION_CLOSE_DEV)       action_close_dev(device_obj);
        actions = device_obj->actions;
        device_obj->actions = 0;
    }
}

// ============================================================
// class_driver_task
// ============================================================

void class_driver_task(void *arg)
{
    class_driver_t driver_obj = {0};
    usb_host_client_handle_t class_driver_client_hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");

    SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();
    if (mux_lock == NULL) {
        ESP_LOGE(TAG, "Unable to create class driver mutex");
        vTaskSuspend(NULL);
        return;
    }

    // Create aircraft mutex early so display timer and on_msg never see NULL
    if (aircraft_mutex == NULL)
        aircraft_mutex = xSemaphoreCreateMutex();
    rx_pos_init();

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &class_driver_client_hdl));

    driver_obj.constant.mux_lock = mux_lock;
    driver_obj.constant.client_hdl = class_driver_client_hdl;
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        driver_obj.mux_protected.device[i].client_hdl = class_driver_client_hdl;

    s_driver_obj = &driver_obj;

    while (1) {
        if (driver_obj.mux_protected.flags.unhandled_devices) {
            xSemaphoreTake(driver_obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
                if (driver_obj.mux_protected.device[i].actions)
                    class_driver_device_handle(&driver_obj.mux_protected.device[i]);
            }
            driver_obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(driver_obj.constant.mux_lock);
        } else if (driver_obj.mux_protected.flags.shutdown) {
            break;
        } else if (s_reader_owns_events) {
            // Reader task is pumping USB events via bulk/control transfers.
            // We must NOT call usb_host_client_handle_events concurrently.
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            // Use timeout instead of portMAX_DELAY so we stay responsive
            // to device reconnection events (USB host lib may need periodic
            // pumping to complete device enumeration after hot-plug).
            usb_host_client_handle_events(class_driver_client_hdl, pdMS_TO_TICKS(500));
        }
    }

    // If reader task is still running, signal it to stop and wait
    if (s_adsb_reader_task_hdl != NULL) {
        s_adsb_reader_stop = true;
        for (int i = 0; i < 50 && s_adsb_reader_task_hdl != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(class_driver_client_hdl));
    if (mux_lock) vSemaphoreDelete(mux_lock);
    vTaskSuspend(NULL);
}

void class_driver_client_deregister(void)
{
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL) {
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));
}

int adsb_get_aircraft_for_scope(scope_aircraft_t *out, int max_count)
{
    if (!aircraft_mutex || !out || max_count <= 0) return 0;
    xSemaphoreTake(aircraft_mutex, portMAX_DELAY);

    receiver_pos_t rx = adsb_get_receiver_pos();
    int64_t now_us = esp_timer_get_time();
    int count = 0;

    for (int i = 0; i < AIRCRAFT_TABLE_SIZE && count < max_count; i++) {
        aircraft_t *a = &aircraft_table[i];
        if (!a->active) continue;
        int64_t age_us = now_us - a->last_seen;
        if (age_us > 180000000LL) continue; // 180s = 3 minutes, matches webapp EXP

        scope_aircraft_t *s = &out[count];
        s->icao = a->icao;
        memcpy(s->callsign, a->callsign, 9);
        s->altitude = a->altitude;
        s->speed = a->speed;
        s->heading = a->heading;
        s->lat = a->lat;
        s->lon = a->lon;
        s->has_position = a->has_position;
        s->vert_rate = a->vert_rate;
        s->msg_count = a->msg_count;
        s->age_ms = (int32_t)(age_us / 1000);
        s->dist_nm = 0;
        s->bearing_deg = 0;

        if (a->has_position && rx.fix_valid) {
            double dist_km, brg;
            haversine(rx.lat, rx.lon, a->lat, a->lon, &dist_km, &brg);
            s->dist_nm = dist_km * 0.539957;
            s->bearing_deg = brg;
        }
        count++;
    }

    xSemaphoreGive(aircraft_mutex);
    return count;
}
