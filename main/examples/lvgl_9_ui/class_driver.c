/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "class_driver.h"
// #include <stdlib.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "esp_log.h"
// #include "usb/usb_host.h"
// #include "rtl-sdr.h"
// #include "mode-s.h"


#define RTLSDR_BUF_LEN (16384 + 512)

#define CLIENT_NUM_EVENT_MSG 5


// typedef enum
// {
//     ACTION_OPEN_DEV = (1 << 0),
//     ACTION_GET_DEV_INFO = (1 << 1),
//     ACTION_GET_DEV_DESC = (1 << 2),
//     ACTION_GET_CONFIG_DESC = (1 << 3),
//     ACTION_GET_STR_DESC = (1 << 4),
//     ACTION_CLOSE_DEV = (1 << 5),
// } action_t;

// #define DEV_MAX_COUNT 128

// typedef struct
// {
//     usb_host_client_handle_t client_hdl;
//     uint8_t dev_addr;
//     usb_device_handle_t dev_hdl;
//     action_t actions;
// } usb_device_t;

// typedef struct
// {
//     struct
//     {
//         union
//         {
//             struct
//             {
//                 uint8_t unhandled_devices : 1; /**< Device has unhandled devices */
//                 uint8_t shutdown : 1;          /**<  */
//                 uint8_t reserved6 : 6;         /**< Reserved */
//             };
//             uint8_t val;                    /**< Class drivers' flags value */
//         } flags;                            /**< Class drivers' flags */
//         usb_device_t device[DEV_MAX_COUNT]; /**< Class drivers' static array of devices */
//     } mux_protected;                        /**< Mutex protected members. Must be protected by the Class mux_lock when accessed */

//     struct
//     {
//         usb_host_client_handle_t client_hdl;
//         SemaphoreHandle_t mux_lock; /**< Mutex for protected members */
//     } constant;                     /**< Constant members. Do not change after installation thus do not require a critical section or mutex */
// } class_driver_t;

// static const char *TAG = "CLASS";
// static rtlsdr_dev_t *rtldev = NULL;
// static class_driver_t *s_driver_obj;
// static mode_s_t state;

#include <math.h>

// CPR position cache
typedef struct {
    uint32_t icao;
    int even_valid, odd_valid;
    int even_lat, even_lon;
    int odd_lat, odd_lon;
    double lat, lon; // decoded result
} cpr_cache_t;

#define CPR_CACHE_SIZE 128
static cpr_cache_t cpr_cache[CPR_CACHE_SIZE];

static cpr_cache_t *cpr_get(uint32_t icao) {
    int idx = icao % CPR_CACHE_SIZE;
    if (cpr_cache[idx].icao != icao) {
        memset(&cpr_cache[idx], 0, sizeof(cpr_cache_t));
        cpr_cache[idx].icao = icao;
    }
    return &cpr_cache[idx];
}

static int cpr_decode(cpr_cache_t *c) {
    // CPR NL function
    #define NL(lat) ((lat) == 0 ? 59 : (int)(2*M_PI / acos(1 - (1-cos(M_PI/30.0)) / pow(cos(M_PI/180.0*(lat)),2))))

    double lat_even = c->even_lat / 131072.0;
    double lat_odd  = c->odd_lat  / 131072.0;

    int j = (int)floor(59 * lat_even - 60 * lat_odd + 0.5);

    double rlat_even = (360.0 / 60) * (((j % 60) + 60) % 60 + lat_even);
    double rlat_odd  = (360.0 / 59) * (((j % 59) + 59) % 59 + lat_odd);
    if (rlat_even >= 270) rlat_even -= 360;
    if (rlat_odd  >= 270) rlat_odd  -= 360;

    if (NL(rlat_even) != NL(rlat_odd)) return 0; // straddles zone boundary

    double rlat = rlat_even; // use even (most recent if both fresh)
    int nl = NL(rlat);

    double lon_even = c->even_lon / 131072.0;
    double lon_odd  = c->odd_lon  / 131072.0;

    int m = (int)floor(lon_even * (nl - 1) - lon_odd * nl + 0.5);
    int ni = nl > 1 ? nl : 1;
    double rlon = (360.0 / ni) * (((m % ni) + ni) % ni + lon_even);
    if (rlon >= 180) rlon -= 360;

    c->lat = rlat;
    c->lon = rlon;
    return 1;
}

void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    mode_s_decode(self, mm, mm->msg);

    // Raw hex
    int msglen = mm->msgbits / 8;
    for (int i = 0; i < msglen; i++)
        fprintf(stdout, "%02X", mm->msg[i]);
    fprintf(stdout, ";\n");

    // ICAO address (always present)
    uint32_t icao = (uint32_t)(mm->aa1 << 16 | mm->aa2 << 8 | mm->aa3);
    fprintf(stdout, "  ICAO: %06lX\n", (unsigned long)icao);

    // Callsign (DF17 type 1-4)
    if (mm->msgtype == 17 && mm->metype >= 1 && mm->metype <= 4) {
        fprintf(stdout, "  Callsign: %.8s\n", mm->flight);
    }

    // Altitude (DF4, DF20, DF17 airborne position)
    if (mm->altitude) {
        fprintf(stdout, "  Altitude: %d %s\n", mm->altitude,
            mm->unit == MODE_S_UNIT_METERS ? "m" : "ft");
    }

    // CPR position decoding
    if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18) {
        cpr_cache_t *c = cpr_get(icao);
        if (mm->fflag) { // odd
            c->odd_lat = mm->raw_latitude;
            c->odd_lon = mm->raw_longitude;
            c->odd_valid = 1;
        } else { // even
            c->even_lat = mm->raw_latitude;
            c->even_lon = mm->raw_longitude;
            c->even_valid = 1;
        }
        if (c->even_valid && c->odd_valid) {
            if (cpr_decode(c)) {
                fprintf(stdout, "  Position: lat=%.5f lon=%.5f\n", c->lat, c->lon);
            }
        }
    }

    // Velocity (DF17 type 19)
    if (mm->msgtype == 17 && mm->metype == 19) {
        fprintf(stdout, "  Speed: %d kt", mm->velocity);
        if (mm->heading_is_valid)
            fprintf(stdout, "  Heading: %d deg", mm->heading);
        fprintf(stdout, "  VRate: %s%d ft/min\n",
            mm->vert_rate_sign ? "-" : "+",
            mm->vert_rate * 64);
    }

    // Squawk (DF5, DF21)
    if (mm->msgtype == 5 || mm->msgtype == 21) {
        fprintf(stdout, "  Squawk: %04x\n", mm->identity);
    }
}

void demodulate(uint8_t *source, int length)
{
    uint16_t *mag = malloc((length / 2) * sizeof(uint16_t));
    mode_s_compute_magnitude_vector(source, mag, length);
    mode_s_detect(&state, mag, length / 2, on_msg);
    free(mag);
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    int n_read = 0;
    uint8_t *buffer = malloc(DEFAULT_BUF_LENGTH * sizeof(uint8_t));
    switch (event_msg->event)
    {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr = event_msg->new_dev.address;
        int r;
        r = rtlsdr_open(&rtldev, event_msg->new_dev.address, driver_obj->mux_protected.device[event_msg->new_dev.address].client_hdl);
        r = rtlsdr_set_center_freq(rtldev, 1090000000);
        if (r < 0)
            fprintf(stderr, "WARNING: Failed to set center freq.\n");
        else
            fprintf(stderr, "Tuned to %u Hz.\n", 1090000000);

        r = rtlsdr_set_sample_rate(rtldev, 2000000);
        if (r < 0)
            fprintf(stderr, "WARNING: Failed to set sample rate.\n");
        else
            fprintf(stderr, "Sampling at %u S/s.\n", 2000000);

        r = rtlsdr_set_tuner_gain_mode(rtldev, 0.5);
        if (r != 0)
            fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
        else
            fprintf(stderr, "Tuner gain set to automatic.\n");

        r = rtlsdr_reset_buffer(rtldev);
        if (r < 0)
            fprintf(stderr, "WARNING: Failed to reset buffers.\n");

        vTaskDelay(100);

        // Allocate transfer here, after USB host is fully running
        alloc_adsb_transfer();

        ESP_LOGI(TAG, "[APP] Free memory: %ld bytes", esp_get_free_heap_size());
        mode_s_init(&state);

        if (buffer == NULL) {
            ESP_LOGE(TAG, "buffer malloc failed");
            break;
        }

        uint8_t *tbuffer = malloc(MAX_PACKET_SIZE * sizeof(uint8_t));
        if (tbuffer == NULL) {
            ESP_LOGE(TAG, "tbuffer malloc failed");
            free(buffer);
            break;
        }

        while (true)
        {
            for (int i = 0; i < DEFAULT_BUF_LENGTH; i += MAX_PACKET_SIZE)
            {
                r = rtlsdr_read_sync(rtldev, tbuffer, MAX_PACKET_SIZE, &n_read);
                if (r < 0)
                {
                    fprintf(stderr, "WARNING: sync read failed. %d\n", r);
                    break;
                }
                if ((uint32_t)n_read < MAX_PACKET_SIZE)
                {
                    fprintf(stderr, "Short read, samples lost, exiting! %d\n", n_read);
                    break;
                }
                memcpy(&buffer[i], tbuffer, MAX_PACKET_SIZE);
            }
            if (n_read > 0) {
                demodulate(buffer, DEFAULT_BUF_LENGTH);
            }
        }

        free(tbuffer);
        free(buffer);
        break;

    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        {
            if (driver_obj->mux_protected.device[i].dev_hdl == event_msg->dev_gone.dev_hdl)
            {
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

static void action_open_dev(usb_device_t *device_obj)
{
    assert(device_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", device_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(device_obj->client_hdl, device_obj->dev_addr, &device_obj->dev_hdl));
    // Get the device's information next
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
    if (dev_info.parent.dev_hdl)
    {
        usb_device_info_t parent_dev_info;
        ESP_ERROR_CHECK(usb_host_device_info(dev_info.parent.dev_hdl, &parent_dev_info));
        ESP_LOGI(TAG, "\t\tBus addr: %d", parent_dev_info.dev_addr);
        ESP_LOGI(TAG, "\t\tPort: %d", dev_info.parent.port_num);
    }
    else
    {
        ESP_LOGI(TAG, "\t\tPort: ROOT");
    }
    ESP_LOGI(TAG, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    // Get the device descriptor next
    device_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(device_obj->dev_hdl, &dev_desc));
    usb_print_device_descriptor(dev_desc);
    // Get the device's config descriptor next
    device_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    ESP_LOGI(TAG, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl, &config_desc));
    usb_print_config_descriptor(config_desc, NULL);
    // Get the device's string descriptors next
    device_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
    if (dev_info.str_desc_manufacturer)
    {
        ESP_LOGI(TAG, "Getting Manufacturer string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    }
    if (dev_info.str_desc_product)
    {
        ESP_LOGI(TAG, "Getting Product string descriptor");
        usb_print_string_descriptor(dev_info.str_desc_product);
    }
    if (dev_info.str_desc_serial_num)
    {
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

    while (actions)
    {
        if (actions & ACTION_OPEN_DEV)
        {
            action_open_dev(device_obj);
        }
        if (actions & ACTION_GET_DEV_INFO)
        {
            action_get_info(device_obj);
        }
        if (actions & ACTION_GET_DEV_DESC)
        {
            action_get_dev_desc(device_obj);
        }
        if (actions & ACTION_GET_CONFIG_DESC)
        {
            action_get_config_desc(device_obj);
        }
        if (actions & ACTION_GET_STR_DESC)
        {
            action_get_str_desc(device_obj);
        }
        if (actions & ACTION_CLOSE_DEV)
        {
            action_close_dev(device_obj);
        }

        actions = device_obj->actions;
        device_obj->actions = 0;
    }
}

void class_driver_task(void *arg)
{
    class_driver_t driver_obj = {0};
    usb_host_client_handle_t class_driver_client_hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");

    SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();
    if (mux_lock == NULL)
    {
        ESP_LOGE(TAG, "Unable to create class driver mutex");
        vTaskSuspend(NULL);
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false, // Synchronous clients currently not supported. Set this to false
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
    {
        driver_obj.mux_protected.device[i].client_hdl = class_driver_client_hdl;
    }

    s_driver_obj = &driver_obj;

    while (1)
    {
        // Driver has unhandled devices, handle all devices first
        if (driver_obj.mux_protected.flags.unhandled_devices)
        {
            xSemaphoreTake(driver_obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
            {
                if (driver_obj.mux_protected.device[i].actions)
                {
                    class_driver_device_handle(&driver_obj.mux_protected.device[i]);
                }
            }
            driver_obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(driver_obj.constant.mux_lock);
        }
        else
        {
            // Driver is active, handle client events
            if (driver_obj.mux_protected.flags.shutdown == 0)
            {
                usb_host_client_handle_events(class_driver_client_hdl, portMAX_DELAY);
            }
            else
            {
                // Shutdown the driver
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(class_driver_client_hdl));
    if (mux_lock != NULL)
    {
        vSemaphoreDelete(mux_lock);
    }
    vTaskSuspend(NULL);
}

void class_driver_client_deregister(void)
{
    // Mark all opened devices
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
    {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL)
        {
            // Mark device to close
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            // Set flag
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);

    // Unblock, exit the loop and proceed to deregister client
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));
}
