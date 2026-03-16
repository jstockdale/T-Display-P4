/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class_driver.h"
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
#define AIRCRAFT_TABLE_SIZE   64
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

static FILE *sd_log = NULL;
static char sd_log_filename[64] = {0};  // current log path for rename

static void sd_log_open(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);

    // Sanity check: if time is before 2024-01-01 the RTC hasn't been set
    if (now < 1704067200LL) {
        snprintf(sd_log_filename, sizeof(sd_log_filename),
            "/sdcard/adsb_boot%llu.csv",
            (unsigned long long)(esp_timer_get_time() / 1000));
    } else {
        strftime(sd_log_filename, sizeof(sd_log_filename),
            "/sdcard/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);
    }

    sd_log = fopen(sd_log_filename, "w");
    if (sd_log) {
        fprintf(sd_log, "timestamp_utc,raw_msg,icao,callsign,altitude_ft,speed_kt,"
                        "heading_deg,vrate_fpm,lat,lon,squawk,"
                        "rx_lat,rx_lon,range_km,bearing_deg,"
                        "rx_sats,rx_hdop\n");
        fflush(sd_log);
        fsync(fileno(sd_log));
    }

    if (sd_log) {
        ESP_LOGI(TAG, "SD log: %s", sd_log_filename);
    } else {
        ESP_LOGW(TAG, "Failed to open SD log: %s", sd_log_filename);
    }
}

// Rename the current log file from boot-numbered to UTC-timestamped.
// Called once from the GPS task when the system clock is first set.
void sd_log_rename_with_time(void) {
    if (!sd_log) return;

    // Only rename if current name is a boot-numbered file
    if (strstr(sd_log_filename, "adsb_boot") == NULL) return;

    time_t now;
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now, &timeinfo);

    // Don't rename if clock still isn't set
    if (now < 1704067200LL) return;

    char new_filename[64];
    strftime(new_filename, sizeof(new_filename),
        "/sdcard/adsb_%Y-%m-%dT%H%M%SZ.csv", &timeinfo);

    // Close, rename, reopen in append mode
    fflush(sd_log);
    fsync(fileno(sd_log));
    fclose(sd_log);
    sd_log = NULL;

    if (rename(sd_log_filename, new_filename) == 0) {
        ESP_LOGI(TAG, "SD log renamed: %s → %s", sd_log_filename, new_filename);
        strncpy(sd_log_filename, new_filename, sizeof(sd_log_filename));
    } else {
        ESP_LOGW(TAG, "SD log rename failed, reopening original");
    }

    sd_log = fopen(sd_log_filename, "a");
    if (!sd_log) {
        ESP_LOGE(TAG, "Failed to reopen SD log: %s", sd_log_filename);
    }
}

// Format current time as ISO-8601 UTC string for CSV logging.
// Falls back to boot-time microseconds if wall clock not set.
static int sd_log_write_timestamp(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    gmtime_r(&tv.tv_sec, &tm_info);

    if (tm_info.tm_year + 1900 >= 2024) {
        return fprintf(sd_log, "%04d-%02d-%02dT%02d:%02d:%02d.%03ldZ",
            tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
            tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
            (long)(tv.tv_usec / 1000));
    } else {
        int64_t boot_ms = esp_timer_get_time() / 1000;
        return fprintf(sd_log, "boot+%lld.%03lld",
            (long long)(boot_ms / 1000), (long long)(boot_ms % 1000));
    }
}

static void sd_log_aircraft(aircraft_t *ac, const struct mode_s_msg *mm) {
    // Lazy open: retry every 5 seconds if the log isn't open yet
    // (SD card may mount late or not be available at boot)
    if (!sd_log) {
        static int64_t last_retry = 0;
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_retry > 5000000LL) {  // 5 seconds
            last_retry = now_us;
            sd_log_open();
        }
        if (!sd_log) return;
    }
    receiver_pos_t rx = adsb_get_receiver_pos();
    double range_km = 0, bearing = 0;
    if (rx.fix_valid && ac->has_position)
        haversine(rx.lat, rx.lon, ac->lat, ac->lon, &range_km, &bearing);

    // Timestamp
    sd_log_write_timestamp();

    // Raw hex message (second column)
    fprintf(sd_log, ",");
    if (mm) {
        int msglen = mm->msgbits / 8;
        for (int i = 0; i < msglen; i++)
            fprintf(sd_log, "%02X", mm->msg[i]);
    }

    // Decoded fields
    fprintf(sd_log,
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

    fflush(sd_log);
    fsync(fileno(sd_log));
}

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
    mode_s_decode(self, mm, mm->msg);

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

    // Altitude
    if (mm->altitude) {
        ac->altitude = mm->altitude;
    }

    // CPR position (DF17, ME 9-18)
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

    // Velocity (DF17, ME 19)
    if (mm->msgtype == 17 && mm->metype == 19) {
        ac->speed     = mm->velocity;
        ac->heading   = mm->heading_is_valid ? mm->heading : ac->heading;
        ac->vert_rate = mm->vert_rate_sign
                      ? -(mm->vert_rate * 64)
                      :  (mm->vert_rate * 64);
    }

    // Squawk (DF5, DF21)
    if (mm->msgtype == 5 || mm->msgtype == 21) {
        ac->squawk = mm->identity;
    }

    // --- Print one-line summary with full known state ---

    // Timestamp
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_info;
    gmtime_r(&tv.tv_sec, &tm_info);
    if (tm_info.tm_year + 1900 >= 2024) {
        fprintf(stdout, "[%04d-%02d-%02d %02d:%02d:%02d.%03ldZ] ",
            tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
            tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
            (long)(tv.tv_usec / 1000));
    } else {
        int64_t boot_ms = esp_timer_get_time() / 1000;
        fprintf(stdout, "[boot+%lld.%03lld] ", boot_ms / 1000, boot_ms % 1000);
    }

    // ICAO + callsign
    fprintf(stdout, "%06lX", (unsigned long)icao);
    if (ac->callsign[0])
        fprintf(stdout, " %-8s", ac->callsign);

    // Altitude + velocity
    if (ac->altitude)
        fprintf(stdout, " %5dft", ac->altitude);
    if (ac->speed)
        fprintf(stdout, " %3dkt %03d°", ac->speed, ac->heading);
    if (ac->vert_rate)
        fprintf(stdout, " %+dfpm", ac->vert_rate);

    // Position + distance from receiver
    if (ac->has_position) {
        fprintf(stdout, " (%.4f%c,%.4f%c)",
            fabs(ac->lat), ac->lat >= 0 ? 'N' : 'S',
            fabs(ac->lon), ac->lon >= 0 ? 'E' : 'W');
        receiver_pos_t rx = adsb_get_receiver_pos();
        if (rx.fix_valid) {
            double dist_km, brg;
            haversine(rx.lat, rx.lon, ac->lat, ac->lon, &dist_km, &brg);
            double dist_nm = dist_km * 0.539957;
            fprintf(stdout, " %.1fnm@%03.0f°", dist_nm, brg);
        }
    }

    // Squawk
    if (ac->squawk)
        fprintf(stdout, " SQK:%04X", ac->squawk);

    // Raw hex (compact, at end)
    fprintf(stdout, " [");
    int msglen = mm->msgbits / 8;
    for (int i = 0; i < msglen; i++)
        fprintf(stdout, "%02X", mm->msg[i]);
    fprintf(stdout, "]\n");

    sd_log_aircraft(ac, mm);

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

    ESP_LOGI(TAG, "[APP] Free memory: %ld bytes", esp_get_free_heap_size());
    mode_s_init(&state);
    sd_log_open();

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
        }
        if (!read_ok) {
            // Back off briefly on read errors before retrying
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    free(s_mag_buf);
    s_mag_buf = NULL;
    free(tbuffer);
    free(buffer);

done:
    if (sd_log) { fclose(sd_log); sd_log = NULL; }
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
            usb_host_client_handle_events(class_driver_client_hdl, portMAX_DELAY);
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
