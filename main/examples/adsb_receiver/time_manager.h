/**
 * time_manager.h — Unified time source management.
 *
 * Coordinates GPS NMEA, NTP, PCF8563 RTC, and (future) GPS PPS to maintain
 * the most accurate system clock possible. Designed so adding PPS discipline
 * later requires minimal changes.
 *
 * Source hierarchy:
 *
 *   Source        Stratum  Est. accuracy  Notes
 *   ────────────  ───────  ─────────────  ──────────────────────────────
 *   GPS PPS       0        ~100ns         Future: hardware interrupt on GPIO
 *   NTP           2        ~30-100ms      WiFi required, configurable server
 *   GPS NMEA      3        ~1-2s          Serial pipeline delay (no PPS)
 *   PCF8563 RTC   16       ~1-5s          Boot seed for TLS; not a sync source
 *   Unsync'd      —        unbounded      Boot with no source yet
 *
 * Boot behavior:
 *   System clock starts at epoch 0. Log lines use [boot+elapsed] format.
 *   On first RTC read, the POSIX clock is seeded from the RTC so that TLS
 *   certificate validation works before GPS/NTP is available. This does NOT
 *   change the log format — logs stay in [boot+elapsed] until a trusted
 *   source (GPS, NTP, or PPS — stratum ≤ 3) syncs, at which point logs
 *   switch to UTC. The RTC is written to when GPS/NTP updates arrive,
 *   keeping it fresh for the next boot seed.
 *
 * How PPS will integrate (future):
 *   1. GPIO edge interrupt on PPS rising edge → captures esp_timer_get_time()
 *   2. NMEA $GPRMC provides the second value → PPS edge marks exact top-of-second
 *   3. time_manager_pps_pulse(timestamp_us) called from ISR via task notification
 *   4. Manager computes offset between local clock and true UTC second boundary
 *   5. Over many PPS pulses: estimate local oscillator drift (ppb)
 *   6. Apply continuous frequency correction via adjtime() or manual compensation
 *   7. For MLAT: expose time_manager_get_precise_us() → UTC microseconds with
 *      estimated error bounds, accounting for oscillator drift since last PPS
 *
 * Part of ADS-B Scope — T-Display-P4.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <sys/time.h>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TM_TAG = "TIME";

// ─── GPS Integrity Modes ─────────────────────────────────────────────────────

typedef enum {
    GPS_INTEGRITY_BASIC   = 0,  // garbled NMEA guard (>1 hr reject, 5-consecutive override)
    GPS_INTEGRITY_STRICT  = 1,  // drift rate guard + NTP cross-check, no override
    // Future modes: 2=paranoid (position consistency), 3=MLAT-grade, ...
} gps_integrity_mode_t;

// ─── Source Identifiers ──────────────────────────────────────────────────────

typedef enum {
    TIME_SRC_NONE    = 0,
    TIME_SRC_RTC     = 1,   // PCF8563 hardware RTC
    TIME_SRC_NTP     = 2,   // SNTP over WiFi
    TIME_SRC_GPS     = 3,   // GPS NMEA serial
    TIME_SRC_PPS     = 4,   // GPS PPS hardware interrupt (future)
} time_source_t;

static inline const char *time_source_name(time_source_t src) {
    switch (src) {
        case TIME_SRC_RTC:  return "RTC";
        case TIME_SRC_NTP:  return "NTP";
        case TIME_SRC_GPS:  return "GPS";
        case TIME_SRC_PPS:  return "PPS";
        default:            return "NONE";
    }
}

// ─── Per-source State ────────────────────────────────────────────────────────

typedef struct {
    time_source_t source;
    bool          active;             // source is producing updates
    int64_t       last_update_us;     // esp_timer_get_time() of last correction
    int64_t       last_offset_us;     // signed offset applied (system was behind if positive)
    uint32_t      est_accuracy_us;    // estimated accuracy in microseconds
    uint32_t      update_count;       // total corrections from this source
    uint8_t       stratum;            // NTP stratum (0=PPS, 1=GPS, 2-3=NTP, 16=RTC)
    uint8_t       consecutive_rejects; // consecutive large-offset rejections (sanity guard)
} time_source_state_t;

// ─── PPS State (future, pre-allocated) ───────────────────────────────────────

typedef struct {
    bool     enabled;                 // PPS GPIO interrupt registered
    uint8_t  gpio;                    // PPS input GPIO
    int64_t  last_edge_us;            // esp_timer_get_time() at last PPS rising edge
    int64_t  last_second_utc;         // UTC second from NMEA paired with last edge
    double   drift_ppb;              // estimated oscillator drift in parts-per-billion
    uint32_t drift_samples;           // number of PPS pairs used in drift estimate
    double   drift_sum;              // running sum for drift averaging
    bool     disciplined;             // true once drift estimate is stable (>60 samples)
} pps_state_t;

// ─── Manager State ───────────────────────────────────────────────────────────

#define TIME_MAX_SOURCES  4
#define TIME_NTP_POLL_MIN_S    300    // 5 minutes minimum
#define TIME_NTP_POLL_MAX_S   3600    // 1 hour maximum
#define TIME_STALE_THRESHOLD_S  600   // source considered stale after 10 min no update

// Basic mode: garbled NMEA guard
#define TIME_BASIC_MAX_S      3600    // reject corrections >1 hour after initial sync
#define TIME_BASIC_OVERRIDE      5    // accept after N consecutive rejections

// Strict mode: drift rate guard + NTP cross-check
#define TIME_STRICT_DRIFT_PPM     100    // max crystal drift (generous — real quartz is 20–50)
#define TIME_STRICT_BASELINE_US   500000 // 500ms baseline for serial jitter
#define TIME_STRICT_NTP_DISAGREE_S  5    // reject GPS if NTP disagrees by more than this

typedef struct {
    time_source_state_t sources[TIME_MAX_SOURCES]; // indexed by time_source_t - 1
    time_source_t       active_source;              // currently governing source
    bool                synced;                     // at least one source has set the clock
    bool                trusted;                    // a quality source (GPS/NTP/PPS, stratum ≤3) has synced
                                                    // — drives log timestamp transition from [boot+elapsed] to UTC
    bool                rtc_seeded;                 // POSIX clock seeded from RTC (for TLS, not logs)
    int64_t             boot_time_us;               // esp_timer when first sync happened

    // Callback: fires when a trusted correction is applied to the system clock.
    // Use to sync external RTC, update displays, etc.
    void              (*on_correction)(time_source_t source);

    // GPS integrity
    gps_integrity_mode_t integrity_mode;            // set from device_settings
    int64_t             last_accepted_utc_us;       // UTC microseconds of last accepted correction
    int64_t             last_accepted_mono_us;      // esp_timer when last correction was accepted

    // NTP config
    char    ntp_server[64];
    int32_t ntp_poll_s;               // poll interval in seconds

    // PPS (pre-allocated for future)
    pps_state_t pps;

    // RTC write-back tracking
    int64_t last_rtc_writeback_us;    // when we last wrote to PCF8563
} time_manager_t;

static time_manager_t s_tm = {};

// ─── Helpers ─────────────────────────────────────────────────────────────────

static inline time_source_state_t *tm_get_source(time_source_t src) {
    if (src < TIME_SRC_RTC || src > TIME_SRC_PPS) return NULL;
    return &s_tm.sources[src - 1];
}

static inline bool tm_source_is_stale(time_source_state_t *ss) {
    if (!ss->active || ss->update_count == 0) return true;
    int64_t age_us = esp_timer_get_time() - ss->last_update_us;
    return age_us > (int64_t)TIME_STALE_THRESHOLD_S * 1000000LL;
}

// Select the best non-stale source by stratum, then by accuracy
static inline time_source_t tm_select_best(void) {
    time_source_t best = TIME_SRC_NONE;
    uint8_t best_stratum = 255;
    uint32_t best_accuracy = UINT32_MAX;

    for (int i = 0; i < TIME_MAX_SOURCES; i++) {
        time_source_state_t *ss = &s_tm.sources[i];
        if (!ss->active || ss->update_count == 0) continue;
        if (tm_source_is_stale(ss)) continue;

        // Lower stratum wins; ties broken by lower accuracy value
        if (ss->stratum < best_stratum ||
            (ss->stratum == best_stratum && ss->est_accuracy_us < best_accuracy)) {
            best = ss->source;
            best_stratum = ss->stratum;
            best_accuracy = ss->est_accuracy_us;
        }
    }
    return best;
}

// ─── Initialization ──────────────────────────────────────────────────────────

/**
 * Initialize the time manager. Call once at boot, before any source starts.
 * ntp_server: NULL or "" to use default "pool.ntp.org"
 * ntp_poll_s: 0 to use default (300s = 5 min)
 */
static inline void time_manager_init(const char *ntp_server, int32_t ntp_poll_s) {
    memset(&s_tm, 0, sizeof(s_tm));

    for (int i = 0; i < TIME_MAX_SOURCES; i++) {
        s_tm.sources[i].source  = (time_source_t)(i + 1);
        s_tm.sources[i].stratum = 16; // default worst
    }

    // Set strata
    tm_get_source(TIME_SRC_RTC)->stratum = 16;
    tm_get_source(TIME_SRC_NTP)->stratum = 2;
    // GPS-over-serial (no PPS) has ~1-2s pipeline delay from fix to NMEA parse.
    // Stratum 3 means it's useful when WiFi/NTP is unavailable, but NTP (stratum 2)
    // wins when both are active. PPS (stratum 0) would override everything.
    tm_get_source(TIME_SRC_GPS)->stratum = 3;
    tm_get_source(TIME_SRC_PPS)->stratum = 0;

    // NTP config
    if (ntp_server && ntp_server[0]) {
        strncpy(s_tm.ntp_server, ntp_server, sizeof(s_tm.ntp_server) - 1);
    } else {
        strncpy(s_tm.ntp_server, "pool.ntp.org", sizeof(s_tm.ntp_server) - 1);
    }
    s_tm.ntp_poll_s = (ntp_poll_s >= TIME_NTP_POLL_MIN_S && ntp_poll_s <= TIME_NTP_POLL_MAX_S)
                       ? ntp_poll_s : TIME_NTP_POLL_MIN_S;

    ESP_LOGI(TM_TAG, "Time manager initialized (NTP: %s, poll: %lds)",
             s_tm.ntp_server, (long)s_tm.ntp_poll_s);
}

/**
 * Set GPS integrity mode. Call after time_manager_init() with the
 * value from device_settings. Can also be changed at runtime.
 */
static inline void time_manager_set_integrity_mode(gps_integrity_mode_t mode) {
    s_tm.integrity_mode = mode;
    ESP_LOGI(TM_TAG, "GPS integrity mode: %s",
             mode == GPS_INTEGRITY_STRICT ? "strict (drift guard + NTP cross-check)"
                                          : "basic (garbled NMEA guard)");
}

static inline gps_integrity_mode_t time_manager_get_integrity_mode(void) {
    return s_tm.integrity_mode;
}

/**
 * Register a callback that fires when a trusted clock correction is applied.
 * Use to sync external RTC, update displays, etc. The callback receives
 * the source that triggered the correction (NTP, GPS, PPS).
 * Set to NULL to unregister.
 */
static inline void time_manager_set_correction_cb(void (*cb)(time_source_t)) {
    s_tm.on_correction = cb;
}

// ─── Core Update ─────────────────────────────────────────────────────────────

/**
 * Report a time correction from a source.
 *
 * utc_time:     the source's idea of current UTC (struct timeval)
 * source:       which source is reporting
 * accuracy_us:  estimated accuracy of this measurement in microseconds
 *               (GPS ~150000, NTP ~50000, RTC ~2000000, PPS ~1)
 *
 * The manager decides whether to apply the correction based on source
 * priority and whether a better source is currently active.
 *
 * Returns true if the correction was applied to the system clock.
 */
static inline bool time_manager_update(time_source_t source,
                                        struct timeval *utc_time,
                                        uint32_t accuracy_us) {
    time_source_state_t *ss = tm_get_source(source);
    if (!ss) return false;

    // Get current system time for offset calculation
    struct timeval now;
    gettimeofday(&now, NULL);
    int64_t now_us  = (int64_t)now.tv_sec * 1000000LL + now.tv_usec;
    int64_t src_us  = (int64_t)utc_time->tv_sec * 1000000LL + utc_time->tv_usec;
    int64_t offset_us = src_us - now_us;

    // Update source state
    ss->active           = true;
    ss->last_update_us   = esp_timer_get_time();
    ss->last_offset_us   = offset_us;
    ss->est_accuracy_us  = accuracy_us;
    ss->update_count++;

    // Decide whether to apply
    time_source_t best = tm_select_best();

    // Only apply if this source is the current best
    if (best != source) {
        ESP_LOGD(TM_TAG, "%s reports offset %+lldus but %s is active (stratum %d < %d)",
                 time_source_name(source), (long long)offset_us,
                 time_source_name(best),
                 tm_get_source(best)->stratum, ss->stratum);
        return false;
    }

    // Skip tiny corrections (< 10ms) to avoid jitter — unless this is the first sync
    if (s_tm.synced && llabs(offset_us) < 10000) {
        ss->consecutive_rejects = 0;
        return false;
    }

    // ── GPS integrity guard (post-initial-sync only) ──
    if (s_tm.synced) {
        if (s_tm.integrity_mode == GPS_INTEGRITY_STRICT) {
            // ── Strict: drift rate guard ──
            // Max plausible offset = elapsed × drift_ppm + baseline jitter.
            // A garbled 2095 date (~2e9s) fails instantly. A gradual spoof
            // shifting >100 ppm fails within one update cycle.
            int64_t mono_now = esp_timer_get_time();
            int64_t elapsed_us = mono_now - s_tm.last_accepted_mono_us;
            if (elapsed_us < 0) elapsed_us = 0; // monotonic wraparound guard
            int64_t max_offset_us = (elapsed_us / 10000) * TIME_STRICT_DRIFT_PPM
                                  + TIME_STRICT_BASELINE_US;

            if (llabs(offset_us) > max_offset_us) {
                ESP_LOGW(TM_TAG, "%s: drift guard rejected %+.1fs "
                         "(max plausible %.3fs after %.1fs elapsed)",
                         time_source_name(source), offset_us / 1000000.0,
                         max_offset_us / 1000000.0, elapsed_us / 1000000.0);
                return false;
            }

            // ── Strict: NTP cross-check (when available) ──
            // If NTP is active and recent, reject GPS if it disagrees with NTP
            if (source == TIME_SRC_GPS) {
                time_source_state_t *ntp = tm_get_source(TIME_SRC_NTP);
                if (ntp && ntp->active && ntp->update_count > 0
                    && !tm_source_is_stale(ntp)) {
                    // NTP last saw system clock at offset ntp->last_offset_us.
                    // GPS now proposes offset_us. If they disagree significantly, reject GPS.
                    int64_t disagreement = llabs(offset_us - ntp->last_offset_us);
                    if (disagreement > (int64_t)TIME_STRICT_NTP_DISAGREE_S * 1000000LL) {
                        ESP_LOGW(TM_TAG, "%s: NTP cross-check failed "
                                 "(GPS offset %+.1fs, NTP offset %+.1fs, delta %.1fs)",
                                 time_source_name(source),
                                 offset_us / 1000000.0,
                                 ntp->last_offset_us / 1000000.0,
                                 disagreement / 1000000.0);
                        return false;
                    }
                }
            }
        } else {
            // ── Basic: reject >1 hour, accept after N consecutive ──
            if (llabs(offset_us) > (int64_t)TIME_BASIC_MAX_S * 1000000LL) {
                ss->consecutive_rejects++;
                if (ss->consecutive_rejects < TIME_BASIC_OVERRIDE) {
                    ESP_LOGW(TM_TAG, "%s: rejected implausible correction %+.1fs "
                             "(reject %u/%u — will accept if persistent)",
                             time_source_name(source), offset_us / 1000000.0,
                             (unsigned)ss->consecutive_rejects,
                             (unsigned)TIME_BASIC_OVERRIDE);
                    return false;
                }
                ESP_LOGW(TM_TAG, "%s: accepting large correction %+.1fs "
                         "after %u consecutive reports",
                         time_source_name(source), offset_us / 1000000.0,
                         (unsigned)ss->consecutive_rejects);
                ss->consecutive_rejects = 0;
            } else {
                ss->consecutive_rejects = 0;
            }
        }
    }

    // Apply the correction
    settimeofday(utc_time, NULL);

    bool was_synced = s_tm.synced;
    s_tm.synced = true;
    s_tm.active_source = source;
    s_tm.last_accepted_utc_us = src_us;
    s_tm.last_accepted_mono_us = esp_timer_get_time();
    if (!was_synced) {
        s_tm.boot_time_us = s_tm.last_accepted_mono_us;
    }
    // Quality source (stratum ≤ 3: PPS/GPS/NTP) — trusted for log timestamps.
    // RTC (stratum 16) seeds POSIX clock for TLS but does NOT flip this flag.
    if (ss->stratum <= 3 && !s_tm.trusted) {
        s_tm.trusted = true;
        ESP_LOGI(TM_TAG, "Trusted time acquired from %s — logs switching to UTC",
                 time_source_name(source));
    }

    // Notify external RTC sync (if registered) — only for trusted sources
    if (ss->stratum <= 3 && s_tm.on_correction) {
        s_tm.on_correction(source);
    }

    // Log significant corrections
    if (llabs(offset_us) > 100000 || !was_synced) {
        ESP_LOGI(TM_TAG, "%s: clock corrected by %+.3fs (accuracy ~%.1fms, count=%lu)",
                 time_source_name(source),
                 offset_us / 1000000.0,
                 accuracy_us / 1000.0,
                 (unsigned long)ss->update_count);
    }

    // Cross-check: warn if sources disagree significantly
    for (int i = 0; i < TIME_MAX_SOURCES; i++) {
        time_source_state_t *other = &s_tm.sources[i];
        if (other->source == source || !other->active || other->update_count == 0) continue;
        if (tm_source_is_stale(other)) continue;

        // Compare: if another recent source disagrees by >500ms, warn
        int64_t disagreement = llabs(ss->last_offset_us - other->last_offset_us);
        if (disagreement > 500000) {
            ESP_LOGW(TM_TAG, "Time source disagreement: %s vs %s = %.3fs "
                     "(possible spoofing or network issue)",
                     time_source_name(source), time_source_name(other->source),
                     disagreement / 1000000.0);
        }
    }

    return true;
}

// ─── NTP Integration ─────────────────────────────────────────────────────────

static void tm_sntp_sync_cb(struct timeval *tv) {
    // SNTP callback — feed into the manager
    bool applied = time_manager_update(TIME_SRC_NTP, tv, 50000); // ~50ms accuracy
    if (!applied) {
        ESP_LOGD(TM_TAG, "NTP sync received but not applied (better source active)");
    }
}

/**
 * Start NTP polling. Call after WiFi is connected.
 * Respects the configured server and poll interval.
 */
static inline void time_manager_start_ntp(void) {
    time_source_state_t *ss = tm_get_source(TIME_SRC_NTP);
    ss->active = true;

    ESP_LOGI(TM_TAG, "Starting NTP: server=%s poll=%lds",
             s_tm.ntp_server, (long)s_tm.ntp_poll_s);

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, s_tm.ntp_server);
    esp_sntp_set_sync_interval((uint32_t)s_tm.ntp_poll_s * 1000);
    esp_sntp_set_time_sync_notification_cb(tm_sntp_sync_cb);
    esp_sntp_init();
}

/**
 * Stop NTP polling. Call when WiFi disconnects or before MSC mode.
 */
static inline void time_manager_stop_ntp(void) {
    if (esp_sntp_enabled()) {
        esp_sntp_stop();
    }
    time_source_state_t *ss = tm_get_source(TIME_SRC_NTP);
    ss->active = false;
    ESP_LOGI(TM_TAG, "NTP stopped");
}

/**
 * Reconfigure NTP server and/or poll interval at runtime.
 * Takes effect on next poll cycle (or immediately if restarted).
 */
static inline void time_manager_set_ntp_config(const char *server, int32_t poll_s) {
    if (server && server[0]) {
        strncpy(s_tm.ntp_server, server, sizeof(s_tm.ntp_server) - 1);
        s_tm.ntp_server[sizeof(s_tm.ntp_server) - 1] = '\0';
    }
    if (poll_s >= TIME_NTP_POLL_MIN_S && poll_s <= TIME_NTP_POLL_MAX_S) {
        s_tm.ntp_poll_s = poll_s;
    }

    // If NTP is running, restart with new config
    if (esp_sntp_enabled()) {
        time_manager_stop_ntp();
        time_manager_start_ntp();
    }
}

// ─── GPS Integration ─────────────────────────────────────────────────────────

/**
 * Report a GPS NMEA time fix. Call from the GPS task on each valid RMC/GGA.
 *
 * utc_sec:      UTC epoch seconds from NMEA parse (integer, floored)
 * serial_latency_us: estimated total delay from fix to processing, including:
 *                    - ~500ms average integer-second truncation (0-999ms uniform)
 *                    - ~1600ms pipeline (L76K processing + UART + NMEA parse + task)
 *                    Measured against NTP ground truth: ~2000ms total.
 *
 * The manager compensates for serial latency and decides whether to apply.
 */
static inline bool time_manager_gps_update(time_t utc_sec, int32_t serial_latency_us) {
    struct timeval tv;
    // Normalize: tv_usec must be 0–999999 for settimeofday()
    tv.tv_sec  = utc_sec + serial_latency_us / 1000000;
    tv.tv_usec = serial_latency_us % 1000000;

    // Accuracy is bounded by integer truncation jitter (~±500ms)
    // plus pipeline variance (~±200ms)
    uint32_t accuracy = 700000; // ~700ms

    return time_manager_update(TIME_SRC_GPS, &tv, accuracy);
}

/**
 * Mark GPS source as inactive (loss of fix).
 */
static inline void time_manager_gps_lost_fix(void) {
    time_source_state_t *ss = tm_get_source(TIME_SRC_GPS);
    ss->active = false;
    ESP_LOGD(TM_TAG, "GPS fix lost — NTP/RTC may take over");
}

// ─── RTC Integration ─────────────────────────────────────────────────────────
//
// The RTC provides two functions:
//
// 1. Boot seed (time_manager_rtc_seed): Sets the POSIX clock from the PCF8563
//    so that TLS certificate validation works before GPS/NTP is available.
//    Does NOT mark the system as synced or trusted — log timestamps stay in
//    [boot+elapsed] format. Only seeds if the RTC looks sane (>2024) and the
//    POSIX clock is still near epoch.
//
// 2. Write-back: GPS/NTP write good time to the RTC for the display clock
//    and for the next boot seed. The RTC is never used as a time source for
//    clock discipline — only for the initial TLS bootstrap.

/**
 * Seed POSIX clock from RTC at boot — for TLS cert validation only.
 *
 * Sets the system clock via settimeofday() but does NOT update any time
 * manager source state, synced flag, or trusted flag. Log timestamps
 * remain in [boot+elapsed] format until GPS or NTP provides verified time.
 *
 * utc_sec: approximate UTC epoch from RTC (converted from local time by caller)
 *
 * Returns true if the clock was seeded, false if skipped (already seeded,
 * RTC value implausible, or POSIX clock already set by a better source).
 */
static inline bool time_manager_rtc_seed(time_t utc_sec) {
    if (s_tm.rtc_seeded) return false;           // already done
    if (utc_sec < 1704067200) return false;       // before 2024-01-01 — RTC garbage

    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    if (tv_now.tv_sec > 1704067200) return false; // POSIX clock already set (NTP won race)

    struct timeval tv_rtc = { .tv_sec = utc_sec, .tv_usec = 0 };
    settimeofday(&tv_rtc, NULL);
    s_tm.rtc_seeded = true;

    struct tm t;
    gmtime_r(&utc_sec, &t);
    ESP_LOGI(TM_TAG, "RTC seed: %04d-%02d-%02d %02d:%02d:%02d UTC",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
    return true;
}

/**
 * Report time from PCF8563 RTC as a time source.
 * NOT used in ADS-B Scope — here for completeness.
 * Use time_manager_rtc_seed() for boot TLS seeding instead.
 */
static inline bool time_manager_rtc_update(time_t utc_sec) {
    struct timeval tv = { .tv_sec = utc_sec, .tv_usec = 0 };
    return time_manager_update(TIME_SRC_RTC, &tv, 2000000); // ~2s accuracy
}

/**
 * Write the current system time back to the PCF8563 RTC.
 * Call periodically (every ~30 min) and on clean shutdown.
 * Only writes if a quality source (GPS/NTP) has synced.
 */
static inline bool time_manager_rtc_writeback(void) {
    if (!s_tm.synced) return false;
    if (s_tm.active_source == TIME_SRC_RTC) return false; // don't write RTC from itself

    // Rate limit: at most every 30 minutes
    int64_t now_us = esp_timer_get_time();
    if (s_tm.last_rtc_writeback_us != 0 &&
        (now_us - s_tm.last_rtc_writeback_us) < 30LL * 60 * 1000000) {
        return false;
    }

    s_tm.last_rtc_writeback_us = now_us;
    return true; // caller should do the actual I2C write to PCF8563
}

// ─── PPS Hooks (future — architecture only) ──────────────────────────────────
//
// When PPS hardware is connected:
//
// 1. Register GPIO interrupt:
//    gpio_install_isr_service(0);
//    gpio_set_intr_type(pps_gpio, GPIO_INTR_POSEDGE);
//    gpio_isr_handler_add(pps_gpio, pps_isr, NULL);
//
// 2. ISR captures timestamp and notifies task:
//    static void IRAM_ATTR pps_isr(void *arg) {
//        s_tm.pps.last_edge_us = esp_timer_get_time();  // IRAM-safe
//        BaseType_t wake;
//        vTaskNotifyGiveFromISR(pps_task_handle, &wake);
//        portYIELD_FROM_ISR(wake);
//    }
//
// 3. PPS task pairs edge with NMEA second and feeds manager:
//    void pps_task(void *arg) {
//        for (;;) {
//            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//            int64_t edge = s_tm.pps.last_edge_us;
//            time_t nmea_sec = latest_nmea_utc_second; // from GPS task
//            struct timeval tv = { .tv_sec = nmea_sec, .tv_usec = 0 };
//            time_manager_update(TIME_SRC_PPS, &tv, 1); // ~1µs accuracy
//            time_manager_pps_discipline(edge, nmea_sec);
//        }
//    }
//
// 4. Drift discipline (called from pps_task):

/**
 * Feed a PPS edge for oscillator discipline (future).
 * edge_us:   esp_timer_get_time() captured at PPS rising edge
 * nmea_sec:  the UTC second this edge marks (from NMEA RMC)
 *
 * After sufficient samples, estimates local oscillator drift in ppb.
 * This enables time_manager_get_precise_us() to interpolate between
 * PPS edges with sub-microsecond estimated error.
 */
static inline void time_manager_pps_discipline(int64_t edge_us, time_t nmea_sec) {
    pps_state_t *pps = &s_tm.pps;

    if (pps->last_edge_us != 0 && pps->last_second_utc != 0) {
        // Expected 1 second between PPS edges
        int64_t measured_us = edge_us - pps->last_edge_us;
        int64_t expected_us = (int64_t)(nmea_sec - pps->last_second_utc) * 1000000LL;

        if (expected_us > 0 && llabs(measured_us - expected_us) < 500000) {
            // Drift = (measured - expected) / expected, in ppb
            double drift = ((double)(measured_us - expected_us) / (double)expected_us) * 1e9;
            pps->drift_sum += drift;
            pps->drift_samples++;
            pps->drift_ppb = pps->drift_sum / pps->drift_samples;
            pps->disciplined = (pps->drift_samples >= 60);

            if (pps->drift_samples % 60 == 0) {
                ESP_LOGI(TM_TAG, "PPS discipline: drift=%.1f ppb (%lu samples, %s)",
                         pps->drift_ppb, (unsigned long)pps->drift_samples,
                         pps->disciplined ? "locked" : "acquiring");
            }
        }
    }

    pps->last_edge_us    = edge_us;
    pps->last_second_utc = nmea_sec;
}

/**
 * Get precise UTC time in microseconds, compensated for oscillator drift.
 * For MLAT: this is the timestamp you tag on each Mode S message.
 *
 * If PPS is disciplined: interpolates from last PPS edge using drift-corrected
 * local oscillator. Estimated error: ~1-10 µs.
 *
 * If PPS not available: falls back to gettimeofday(). Estimated error depends
 * on active source (GPS ~150ms, NTP ~50ms, RTC ~2s).
 *
 * Returns: UTC microseconds since epoch
 * *est_error_us: estimated error (output, can be NULL)
 */
static inline int64_t time_manager_get_precise_us(uint32_t *est_error_us) {
    pps_state_t *pps = &s_tm.pps;

    if (pps->disciplined && pps->last_edge_us != 0) {
        // Interpolate from last PPS edge
        int64_t now_local = esp_timer_get_time();
        int64_t since_edge = now_local - pps->last_edge_us;

        // Apply drift correction: true_elapsed = measured * (1 - drift_ppb/1e9)
        double corrected = (double)since_edge * (1.0 - pps->drift_ppb / 1e9);

        int64_t utc_us = pps->last_second_utc * 1000000LL + (int64_t)corrected;
        if (est_error_us) *est_error_us = 5; // ~5 µs when disciplined
        return utc_us;
    }

    // Fallback: system clock
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int64_t utc_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;

    if (est_error_us) {
        time_source_state_t *ss = tm_get_source(s_tm.active_source);
        *est_error_us = ss ? ss->est_accuracy_us : 5000000;
    }
    return utc_us;
}

// ─── Status / Query ──────────────────────────────────────────────────────────

/**
 * Check if a trusted time source (GPS, NTP, or PPS) has synced.
 * Drives the log timestamp transition from [boot+elapsed] to UTC.
 * RTC boot seeding does NOT make this return true.
 */
static inline bool time_manager_is_trusted(void) {
    return s_tm.trusted;
}

/**
 * C-linkage wrapper for time_manager_is_trusted() — used by serial_console.c
 * for log timestamp formatting.
 */
extern "C" bool time_manager_is_trusted_fn(void) {
    return s_tm.trusted;
}

/**
 * Check if we have any time source at all (including RTC seed).
 */
static inline bool time_manager_has_quality_source(void) {
    return s_tm.synced && s_tm.active_source != TIME_SRC_NONE;
}

extern "C" bool gps_clock_is_set(void) {
    time_source_state_t *gps = tm_get_source(TIME_SRC_GPS);
    return gps && gps->active && gps->update_count > 0 && !tm_source_is_stale(gps);
}

/**
 * Get the currently active time source.
 */
static inline time_source_t time_manager_active_source(void) {
    return s_tm.active_source;
}

/**
 * Print status of all time sources to the console.
 */
static inline void time_manager_print_status(void) {
    printf("[TIME] Active source: %s  Synced: %s  Trusted: %s  RTC-seeded: %s\n",
           time_source_name(s_tm.active_source),
           s_tm.synced ? "yes" : "no",
           s_tm.trusted ? "yes" : "no",
           s_tm.rtc_seeded ? "yes" : "no");

    for (int i = 0; i < TIME_MAX_SOURCES; i++) {
        time_source_state_t *ss = &s_tm.sources[i];
        if (ss->update_count == 0 && !ss->active) continue;

        int64_t age_s = (esp_timer_get_time() - ss->last_update_us) / 1000000;
        printf("  %4s: stratum=%d accuracy=~%.1fms updates=%lu last=%llds ago offset=%+.3fs %s%s\n",
               time_source_name(ss->source),
               ss->stratum,
               ss->est_accuracy_us / 1000.0,
               (unsigned long)ss->update_count,
               (long long)age_s,
               ss->last_offset_us / 1000000.0,
               ss->active ? "ACTIVE" : "inactive",
               tm_source_is_stale(ss) ? " STALE" : "");
    }

    if (s_tm.pps.drift_samples > 0) {
        printf("  PPS: drift=%.1f ppb samples=%lu %s\n",
               s_tm.pps.drift_ppb,
               (unsigned long)s_tm.pps.drift_samples,
               s_tm.pps.disciplined ? "LOCKED" : "acquiring");
    }
}
