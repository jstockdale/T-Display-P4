/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "rtl-sdr.h"
#include "mode-s.h"
#include "lvgl.h"


#define CLIENT_NUM_EVENT_MSG 5

#define MAX_PACKET_SIZE 16384
#define DEFAULT_BUF_LENGTH (MAX_PACKET_SIZE * 8)


typedef enum
{
    ACTION_OPEN_DEV = (1 << 0),
    ACTION_GET_DEV_INFO = (1 << 1),
    ACTION_GET_DEV_DESC = (1 << 2),
    ACTION_GET_CONFIG_DESC = (1 << 3),
    ACTION_GET_STR_DESC = (1 << 4),
    ACTION_CLOSE_DEV = (1 << 5),
} action_t;

#define DEV_MAX_COUNT 128

typedef struct
{
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    action_t actions;
} usb_device_t;

typedef struct
{
    struct
    {
        union
        {
            struct
            {
                uint8_t unhandled_devices : 1; /**< Device has unhandled devices */
                uint8_t shutdown : 1;          /**<  */
                uint8_t reserved6 : 6;         /**< Reserved */
            };
            uint8_t val;                    /**< Class drivers' flags value */
        } flags;                            /**< Class drivers' flags */
        usb_device_t device[DEV_MAX_COUNT]; /**< Class drivers' static array of devices */
    } mux_protected;                        /**< Mutex protected members. Must be protected by the Class mux_lock when accessed */

    struct
    {
        usb_host_client_handle_t client_hdl;
        SemaphoreHandle_t mux_lock; /**< Mutex for protected members */
    } constant;                     /**< Constant members. Do not change after installation thus do not require a critical section or mutex */
} class_driver_t;

static const char *TAG = "CLASS";
static rtlsdr_dev_t *rtldev = NULL;
static class_driver_t *s_driver_obj;
static mode_s_t state;

void on_msg(mode_s_t *self, struct mode_s_msg *mm);

void demodulate(uint8_t *source, int length);

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg);

static void action_open_dev(usb_device_t *device_obj);

static void action_get_info(usb_device_t *device_obj);

static void action_get_dev_desc(usb_device_t *device_obj);

static void action_get_config_desc(usb_device_t *device_obj);

static void action_get_str_desc(usb_device_t *device_obj);

static void action_close_dev(usb_device_t *device_obj);

static void class_driver_device_handle(usb_device_t *device_obj);

#ifdef __cplusplus
extern "C" {
#endif

extern void class_driver_task(void *arg);

extern void class_driver_client_deregister(void);

extern void alloc_adsb_transfer(void);

extern void adsb_update_display(lv_obj_t *table);
extern void sd_log_rename_with_time(void);
extern void sd_log_close(void);
extern void sd_log_print_status(void);
extern void sd_clear_dirty_flag(void);

// Receiver position — updated by GPS task, read by ADS-B display/logging
typedef struct {
    double lat;         // decimal degrees, positive = N
    double lon;         // decimal degrees, positive = E
    double alt_m;       // altitude in meters (from GGA, 0 if unavailable)
    int    fix_valid;   // nonzero when we have a valid fix
    int    sats;        // satellite count from GGA
    double hdop;        // horizontal dilution of precision
    int    fix_quality; // GGA fix quality (0=none, 1=GPS, 2=DGPS)
} receiver_pos_t;

extern void adsb_set_receiver_pos(double lat, double lon, double alt_m,
                                  int sats, double hdop, int fix_quality);
extern receiver_pos_t adsb_get_receiver_pos(void);

// ADS-B statistics — updated by on_msg, read by CIT test / status bar
typedef struct {
    uint32_t total_messages;
    float    msg_rate;          // messages per second
    int      active_aircraft;   // aircraft seen within expiry window
    bool     rtlsdr_connected;  // true while reader task is running AND transfer buffer OK
    bool     rtlsdr_error;      // true if device seen but transfer buffer alloc failed
    uint32_t nearest_icao;      // ICAO of nearest aircraft (0 if none)
    double   nearest_dist_nm;   // distance to nearest in nautical miles
    int      nearest_alt;       // altitude of nearest aircraft
    char     nearest_callsign[9]; // callsign of nearest (empty string if none)
} adsb_stats_t;

extern adsb_stats_t adsb_get_stats(void);

// Sort column for on-device aircraft list
typedef enum {
    ADSB_SORT_DIST = 0,  // default
    ADSB_SORT_ICAO,
    ADSB_SORT_CALL,
    ADSB_SORT_ALT,
    ADSB_SORT_SPD,
    ADSB_SORT_HDG,
} adsb_sort_col_t;

extern void adsb_set_sort(int col, bool ascending);

// Format aircraft list into text buffer, sorted by current sort column.
// Returns number of aircraft written.
extern int adsb_format_aircraft_list(char *buf, int bufsize);

// Pre-allocate USB transfer buffers — call after usb_host_install(),
// before class_driver_task starts.  Idempotent (safe to call from rtlsdr_open too).
extern void init_adsb_dev(void);

// Check if USB bulk transfer buffer is allocated and ready
extern bool adsb_transfer_ready(void);

// Enable/disable bias-T power on RTL-SDR antenna port (4.5V DC).
// Safe to call anytime — no-op if RTL-SDR not connected.
extern void adsb_set_bias_tee(bool on);

// Free bulk transfer on disconnect — allows re-alloc on reconnect
extern void free_adsb_transfer(void);

// Aircraft data for scope radar display
typedef struct {
    uint32_t icao;
    char callsign[9];
    int altitude;
    int speed;
    int heading;
    int vert_rate;
    double lat, lon;
    int has_position;
    double dist_nm;
    double bearing_deg;
    uint32_t msg_count;
    int32_t age_ms;      // milliseconds since last seen
} scope_aircraft_t;

// Get active aircraft with computed distance/bearing from receiver.
// Returns number of aircraft copied.
extern int adsb_get_aircraft_for_scope(scope_aircraft_t *out, int max_count);

// Persistent scope trail recording — call after ADS-B init, records at 1Hz from boot.
extern void scope_trail_init(void);

#ifdef __cplusplus
}
#endif