#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_libusb.h"
#include "esp_heap_caps.h"
#include <string.h>

#define RTLSDR_BUF_LEN (16384 + 512)
#define CTRL_TRANSFER_MAX_SIZE 1024  // largest control transfer for RTL-SDR

// DMA RAM reservation — allocated early in app_main() before heap fragmentation,
// freed here right before the USB transfer alloc to guarantee a contiguous hole.
void *g_usb_dma_reservation = NULL;

class_adsb_dev *adsbdev;
void init_adsb_dev()
{
    // Idempotent — may be called early from main.cpp and again from rtlsdr_open()
    if (adsbdev != NULL) return;

    adsbdev = calloc(1, sizeof(class_adsb_dev));
    if (adsbdev == NULL) {
        ESP_LOGE(TAG_ADSB, "init_adsb_dev: calloc failed");
        return;
    }
    adsbdev->is_adsb = true;

    // Pre-allocate control transfer buffer once — eliminates the rapid
    // alloc/free cycle during R820T tuner init that fragments internal
    // heap and corrupts TLSF metadata.
    esp_err_t r = usb_host_transfer_alloc(CTRL_TRANSFER_MAX_SIZE, 0, &adsbdev->ctrl_transfer);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_ADSB, "ctrl_transfer pre-alloc failed: %d", r);
        adsbdev->ctrl_transfer = NULL;
    }
    adsbdev->response_buf = calloc(CTRL_TRANSFER_MAX_SIZE, sizeof(uint8_t));
    if (adsbdev->response_buf == NULL) {
        ESP_LOGE(TAG_ADSB, "response_buf pre-alloc failed");
    }

    // Free the DMA reservation to create a contiguous hole, then immediately
    // allocate the USB bulk transfer into that space.  The reservation was
    // made at the very start of app_main() before peripheral init fragmented
    // the heap.
    if (g_usb_dma_reservation != NULL) {
        heap_caps_free(g_usb_dma_reservation);
        g_usb_dma_reservation = NULL;
        ESP_LOGI(TAG_ADSB, "Released DMA reservation for USB transfer");
    }

    r = usb_host_transfer_alloc(RTLSDR_BUF_LEN + 512, 0, &adsbdev->transfer);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_ADSB, "bulk transfer alloc failed: %d (internal free: %ld)",
                 r, (long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        adsbdev->transfer = NULL;
    } else {
        ESP_LOGI(TAG_ADSB, "bulk transfer alloc success");
    }
}

void alloc_adsb_transfer(void) {
    if (adsbdev == NULL) return;
    if (adsbdev->transfer != NULL) {
        ESP_LOGI(TAG_ADSB, "transfer alloc: already pre-allocated");
        return;
    }
    // Release DMA reservation to free contiguous hole for the transfer
    if (g_usb_dma_reservation) {
        free(g_usb_dma_reservation);
        g_usb_dma_reservation = NULL;
        ESP_LOGI(TAG_ADSB, "Released DMA reservation for USB transfer");
    }
    esp_err_t r = usb_host_transfer_alloc(RTLSDR_BUF_LEN + 512, 0, &adsbdev->transfer);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_ADSB, "transfer alloc failed: %d (internal free: %ld)",
                 r, (long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    } else {
        ESP_LOGI(TAG_ADSB, "bulk transfer alloc success");
    }
}

// Check if bulk transfer buffer is ready for use
bool adsb_transfer_ready(void) {
    return (adsbdev != NULL && adsbdev->transfer != NULL);
}

// Don't free the bulk transfer buffer — it was pre-allocated early when
// internal RAM was contiguous and can't be re-allocated later due to
// fragmentation.  The buffer is just DMA-capable memory; the device_handle
// is set fresh on every esp_libusb_bulk_transfer() call, so it works
// across device reconnects without re-allocation.
void free_adsb_transfer(void) {
    // Intentionally empty — buffer stays allocated for reuse on reconnect
}

void bulk_transfer_read_cb(usb_transfer_t *transfer)
{
    // int in_xfer = transfer->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK;
    // if ((transfer->status == 0) && in_xfer)
    // {
    //     for (int i = 0; i < 10; i++)
    //         fprintf(stdout, "%02X", transfer->data_buffer[i]);
    //     fprintf(stdout, "\n");
    //     // for (int i = 0; i < transfer->actual_num_bytes; i++)
    //     // {
    //     //     adsbdev->response_buf[i] = transfer->data_buffer[i];
    //     // }
    // }

    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->actual_num_bytes;
    //  printf("BULK: Transfer:Read type %d\n", transfer->actual_num_bytes);
    // printf("BULK: Transfer:Read status %d, actual number of bytes transferred %d, databuffer size %d, %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer_size, adsbdev->response_buf[8]);
}

void transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++)
    {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_done = true;
    adsbdev->is_success = transfer->status == 0;
    adsbdev->bytes_transferred = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
    // printf("Transfer:Read type %d %ld \n", transfer->actual_num_bytes, transfer->flags);
    // printf("Transfer:Read status %d, actual number of bytes transferred %d, databuffer size %d, %d\n", transfer->status, transfer->actual_num_bytes, transfer->data_buffer[8], adsbdev->response_buf[8]);
}

int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    if (adsbdev->transfer == NULL) {
        ESP_LOGI(TAG_ADSB, "esp_libusb_bulk_transfer: no transfer allocated");
        return -1;
    }
    // if (adsbdev->transfer != NULL) {
    //     usb_host_transfer_free(adsbdev->transfer);
    //     adsbdev->transfer = NULL;
    // }
    
    // size_t sizePacket = usb_round_up_to_mps(length, 512);
    // esp_err_t r = usb_host_transfer_alloc(sizePacket, 0, &adsbdev->transfer);
    // if (r != ESP_OK)
    // {
    //     ESP_LOGI(TAG_ADSB, "esp_libusb_bulk_transfer a failed with %d", r);
    //     return -1;
    // }

    //ESP_LOGI(TAG_ADSB, "esp_libusb_bulk_transfer submitting %d bytes", length);
    adsbdev->transfer->num_bytes = length;
    adsbdev->transfer->device_handle = driver_obj->dev_hdl;
    adsbdev->transfer->timeout_ms = timeout;
    adsbdev->transfer->bEndpointAddress = endpoint;
    adsbdev->transfer->callback = bulk_transfer_read_cb;
    adsbdev->transfer->context = (void *)&driver_obj;
    adsbdev->is_done = false;
    esp_err_t r = usb_host_transfer_submit(adsbdev->transfer);
    if (r != ESP_OK)
    {
        ESP_LOGI(TAG_ADSB, "esp_libusb_bulk_transfer b failed with %d", r);
        return -1;
    }
    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }
    if (!adsbdev->is_success)
    {
        ESP_LOGI(TAG_ADSB, "esp_libusb_bulk_transfer c failed");
        return -1;
    }
    *transferred = adsbdev->bytes_transferred;
    memcpy(data, adsbdev->transfer->data_buffer, *transferred);
    return 0;
}

int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    size_t sizePacket = sizeof(usb_setup_packet_t) + wLength;

    // Sanity check — control transfer must fit in pre-allocated buffer
    if (sizePacket > CTRL_TRANSFER_MAX_SIZE) {
        ESP_LOGE(TAG_ADSB, "control transfer too large: %u > %d", (unsigned)sizePacket, CTRL_TRANSFER_MAX_SIZE);
        return -1;
    }
    if (adsbdev->ctrl_transfer == NULL || adsbdev->response_buf == NULL) {
        ESP_LOGE(TAG_ADSB, "control transfer buffers not allocated");
        return -1;
    }

    // Reuse pre-allocated ctrl_transfer and response_buf — no alloc/free churn
    memset(adsbdev->response_buf, 0, sizePacket);

    USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t *)adsbdev->ctrl_transfer->data_buffer, bm_req_type, b_request, wValue, wIndex, wLength);
    adsbdev->ctrl_transfer->num_bytes = sizePacket;
    adsbdev->ctrl_transfer->device_handle = driver_obj->dev_hdl;
    adsbdev->ctrl_transfer->timeout_ms = timeout;
    adsbdev->ctrl_transfer->context = (void *)&driver_obj;
    adsbdev->ctrl_transfer->callback = transfer_read_cb;
    adsbdev->is_done = false;

    if (bm_req_type == CTRL_OUT)
    {
        for (uint8_t i = 0; i < wLength; i++)
        {
            adsbdev->ctrl_transfer->data_buffer[sizeof(usb_setup_packet_t) + i] = data[i];
        }
    }
    esp_err_t r = usb_host_transfer_submit_control(driver_obj->client_hdl, adsbdev->ctrl_transfer);
    if (r != ESP_OK)
    {
        ESP_LOGI(TAG_ADSB, "libusb_control_transfer failed with %d", r);
        return -1;
    }

    while (!adsbdev->is_done)
    {
        usb_host_client_handle_events(driver_obj->client_hdl, portMAX_DELAY);
    }
    if (!adsbdev->is_success)
    {
        ESP_LOGI(TAG_ADSB, "libusb_control_transfer failed");
        return -1;
    }
    for (uint8_t i = 0; i < wLength; i++)
    {
        data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
    }
    return adsbdev->bytes_transferred;
}

void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str)
{
    if (str_desc == NULL)
    {
        return;
    }

    for (int i = 0; i < str_desc->bLength / 2; i++)
    {
        str[i] = (char)str_desc->wData[i];
    }
}
