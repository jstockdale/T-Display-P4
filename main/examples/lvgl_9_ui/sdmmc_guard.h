/*
 * SDMMC controller sharing workaround (Espressif Issue #17889)
 *
 * ESP-Hosted WiFi uses SDIO Slot 1 and SD card uses Slot 0 on the
 * same SDMMC controller. Concurrent DMA corrupts heap metadata.
 *
 * Call sdmmc_guard_acquire() before any SD card file I/O and
 * sdmmc_guard_release() after. This suspends ESP-Hosted's
 * sdio_process_rx task to serialize access to the SDMMC controller.
 */
#ifndef SDMMC_GUARD_H
#define SDMMC_GUARD_H

#ifdef __cplusplus
extern "C" {
#endif

void sdmmc_guard_acquire(void);
void sdmmc_guard_release(void);

#ifdef __cplusplus
}
#endif

#endif // SDMMC_GUARD_H
