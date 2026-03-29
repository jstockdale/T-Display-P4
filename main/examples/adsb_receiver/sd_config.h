/*
 * sd_config.h – SD card configuration for ADS-B Scope
 *
 * Manages /sdcard/.adsb-scope.conf with a boot_count counter so
 * ADS-B and Meshy logs from the same boot share the same boot ID.
 *
 * Call sd_config_init() once after SD mount, before log creation.
 */
#ifndef SD_CONFIG_H
#define SD_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_CONFIG_PATH "/sdcard/.adsb-scope.conf"

// Call once after SD card mount. Reads boot_count, increments, writes back.
void sd_config_init(void);

// 8-digit zero-padded boot ID for sortable filenames.
// Returns "00000000" if init hasn't been called.
const char *sd_config_boot_id(void);

#ifdef __cplusplus
}
#endif

#endif // SD_CONFIG_H
