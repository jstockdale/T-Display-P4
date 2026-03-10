/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "rtl-sdr.h"
#include "mode-s.h"

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

#ifdef __cplusplus
}
#endif