/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class_driver.h"
#include "serial_console.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

#define RTLSDR_BUF_LEN        (16384 + 512)
#define CLIENT_NUM_EVENT_MSG  5
#define CPR_MAX_AGE_US        10000000LL   // 10 seconds
#define CPR_CACHE_SIZE        256
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

// Forward declaration — defined below
static void haversine(double lat1, double lon1, double lat2, double lon2,
                      double *out_dist_km, double *out_bearing_deg);

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

    // Find nearest aircraft
    stats.nearest_icao = 0;
    stats.nearest_dist_nm = 0;
    receiver_pos_t rx = adsb_get_receiver_pos();
    if (rx.fix_valid && aircraft_mutex) {
        double best_dist = 1e9;
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
        }
        xSemaphoreGive(aircraft_mutex);
    }

    return stats;
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

// ============================================================
// SD card logging
// ============================================================

static char sd_log_filename[64] = {0};
static int64_t sd_log_last_sync = 0;
static int sd_log_writes = 0;
static bool sd_log_initialized = false;  // header written at least once
#define SD_SYNC_INTERVAL_US  5000000LL   // flush buffer every 5s
#define SD_SYNC_WRITE_COUNT  100         // or every 100 messages

// PSRAM write buffer — messages accumulate here between flushes.
// File stays CLOSED between sync cycles so the FAT32 dirty bit is
// never set when a hard reset occurs.  On reset we lose at most one
// sync interval of buffered data, but macOS mounts without repair.
#define SD_LOG_BUFSIZE  (64 * 1024)
static char *sd_log_buf = NULL;
static int   sd_log_buf_pos = 0;

static void sd_log_pick_filename(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);

    if (now < 1704067200LL) {
        snprintf(sd_log_filename, sizeof(sd_log_filename),
            "/sdcard/adsb_boot%llu.csv",
            (unsigned long long)(esp_timer_get_time() / 1000));
    } else {
        strftime(sd_log_filename, sizeof(sd_log_filename),
            "/sdcard/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);
    }
}

// Write the CSV header to a new file and close it immediately.
static void sd_log_create(void) {
    sd_log_pick_filename();

    FILE *f = fopen(sd_log_filename, "w");
    if (!f) {
        ESP_LOGW(TAG, "Failed to open SD log: %s", sd_log_filename);
        return;
    }
    fprintf(f, "timestamp_utc,raw_msg,icao,callsign,altitude_ft,speed_kt,"
               "heading_deg,vrate_fpm,lat,lon,squawk,"
               "rx_lat,rx_lon,range_km,bearing_deg,"
               "rx_sats,rx_hdop\n");
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    sd_clear_dirty_flag();

    sd_log_initialized = true;
    sd_log_last_sync = esp_timer_get_time();
    sd_log_writes = 0;
    sd_log_buf_pos = 0;
    ESP_LOGI(TAG, "SD log: %s", sd_log_filename);
}

// Flush the PSRAM buffer to disk: open → append → close → clear dirty.
// File is open for only the duration of this call (~10-50ms).
static void sd_log_flush(void) {
    if (!sd_log_initialized || sd_log_buf_pos == 0) return;
    if (sd_log_filename[0] == '\0') return;

    FILE *f = fopen(sd_log_filename, "a");
    if (!f) {
        ESP_LOGW(TAG, "SD flush: cannot open %s", sd_log_filename);
        // Don't discard buffer — retry next cycle
        return;
    }

    size_t written = fwrite(sd_log_buf, 1, sd_log_buf_pos, f);
    fflush(f);
    fsync(fileno(f));
    fclose(f);

    // File is now closed — FatFS has no cached state.
    // Clear dirty bit directly on disk; nothing can overwrite it
    // until the next fopen (which only happens at next flush).
    sd_clear_dirty_flag();

    if ((int)written == sd_log_buf_pos) {
        sd_log_buf_pos = 0;
    } else {
        // Partial write — shift unwritten data to front
        int remain = sd_log_buf_pos - (int)written;
        memmove(sd_log_buf, sd_log_buf + written, remain);
        sd_log_buf_pos = remain;
        ESP_LOGW(TAG, "SD flush: partial write (%d/%d)", (int)written, sd_log_buf_pos + (int)written);
    }

    sd_log_last_sync = esp_timer_get_time();
    sd_log_writes = 0;
}

// Close the log permanently (for clean shutdown/unmount).
void sd_log_close(void) {
    sd_log_flush();  // write any remaining buffered data
    sd_log_initialized = false;
    ESP_LOGI(TAG, "SD log closed: %s", sd_log_filename);
}

// Print current SD log status.
void sd_log_print_status(void) {
    if (sd_log_initialized && sd_log_filename[0]) {
        printf("I (%lu) CLASS: SD log: %s\n",
            (unsigned long)(esp_timer_get_time() / 1000), sd_log_filename);
    }
}

// Rename boot-numbered log to UTC-timestamped name.
// Called once from GPS task when the system clock is first set.
void sd_log_rename_with_time(void) {
    if (!sd_log_initialized) return;
    if (strstr(sd_log_filename, "adsb_boot") == NULL) return;

    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);
    if (now < 1704067200LL) return;

    // Flush any buffered data to the old filename first
    sd_log_flush();

    char new_filename[64];
    strftime(new_filename, sizeof(new_filename),
        "/sdcard/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);

    // File is already closed (flush closes it), so rename is safe
    if (rename(sd_log_filename, new_filename) == 0) {
        ESP_LOGI(TAG, "SD log renamed: %s → %s", sd_log_filename, new_filename);
        strncpy(sd_log_filename, new_filename, sizeof(sd_log_filename));
    } else {
        ESP_LOGW(TAG, "SD log rename failed");
    }
}

// Append a formatted timestamp to the PSRAM buffer.
static void sd_log_buf_timestamp(void) {
    if (!sd_log_buf) return;
    int avail = SD_LOG_BUFSIZE - sd_log_buf_pos;
    if (avail < 40) return;  // need room for timestamp

    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    gmtime_r(&tv.tv_sec, &tm_info);

    int n;
    if (tm_info.tm_year + 1900 >= 2024) {
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
    // Allocate buffer once
    if (!sd_log_buf) {
        sd_log_buf = heap_caps_malloc(SD_LOG_BUFSIZE, MALLOC_CAP_SPIRAM);
        if (!sd_log_buf) return;
        sd_log_buf_pos = 0;
    }

    // Lazy create: retry every 5 seconds if the log hasn't been created yet
    if (!sd_log_initialized) {
        static int64_t last_retry = 0;
        static int64_t last_mount_retry = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_retry > 5000000LL) {
            last_retry = now_us;
            sd_log_create();
            if (!sd_log_initialized && (now_us - last_mount_retry > 30000000LL)) {
                last_mount_retry = now_us;
                extern bool sd_remount(void);
                if (sd_remount()) {
                    sd_log_create();
                }
            }
        }
        if (!sd_log_initialized) return;
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
        ",%06lX,%s,%d,%d,%d,%d,%.5f,%.5f,%04X,%.5f,%.5f,%.1f,%.0f,%d,%.1f\n",
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

    sd_log_writes++;

    // Periodic flush: buffer → disk → close → clear dirty
    int64_t sync_now = esp_timer_get_time();
    if (sd_log_writes >= SD_SYNC_WRITE_COUNT ||
        (sync_now - sd_log_last_sync) > SD_SYNC_INTERVAL_US) {
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
        if (c->even_valid && c->odd_valid) {
            if (cpr_decode(c)) {
                ac->lat = c->lat;
                ac->lon = c->lon;
                ac->has_position = 1;
            }
        }
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

    // Timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    gmtime_r(&tv.tv_sec, &tm_info);
    if (tm_info.tm_year + 1900 >= 2024) {
        int n = snprintf(line + pos, avail, "[%04d-%02d-%02d %02d:%02d:%02d.%03ldZ] ",
            tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
            tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
            (long)(tv.tv_usec / 1000));
        if (n > 0) { pos += n; avail -= n; }
    } else {
        int64_t boot_ms = esp_timer_get_time() / 1000;
        int n = snprintf(line + pos, avail, "[boot+%lld.%03lld] ", boot_ms / 1000, boot_ms % 1000);
        if (n > 0) { pos += n; avail -= n; }
    }

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
        int n = snprintf(line + pos, avail, " SQK:%04X", ac->squawk);
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

    r = rtlsdr_set_center_freq(rtldev, 1090000000);
    if (r < 0) fprintf(stderr, "WARNING: Failed to set center freq.\n");
    else       fprintf(stderr, "Tuned to %u Hz.\n", 1090000000);

    r = rtlsdr_set_sample_rate(rtldev, 2000000);
    if (r < 0) fprintf(stderr, "WARNING: Failed to set sample rate.\n");
    else       fprintf(stderr, "Sampling at %u S/s.\n", 2000000);

    r = rtlsdr_set_tuner_gain_mode(rtldev, 0);  // 0 = automatic gain
    if (r != 0) fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
    else        fprintf(stderr, "Tuner gain set to automatic.\n");

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
    while (!s_adsb_reader_stop) {
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
        abort();
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
