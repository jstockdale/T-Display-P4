/*
 * tz_lookup.h — GPS-based timezone lookup with DST support
 *
 * Given a latitude/longitude and UTC date, returns the local UTC offset
 * in minutes, including DST if applicable.
 *
 * Data is generated offline by generate_tz_data.py from the
 * timezone-boundary-builder dataset and stored as a RLE-compressed
 * 0.1° grid in timezone_data.h (~100-150KB in flash).
 *
 * License: BSD 3-Clause (Off by One / John Stockdale)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Result of a timezone lookup
typedef struct {
    int16_t total_offset_min;   // Total UTC offset in minutes (std + DST if active)
    int16_t std_offset_min;     // Standard (non-DST) offset in minutes
    int16_t dst_offset_min;     // DST offset (0 if DST not active or not applicable)
    bool    has_dst;            // true if this region observes DST
    bool    dst_active;         // true if DST is currently in effect
} tz_result_t;

/**
 * Look up the timezone for a given position and UTC date.
 *
 * @param lat       Latitude in degrees (-90 to +90)
 * @param lon       Longitude in degrees (-180 to +180)
 * @param utc_year  UTC year (e.g. 2026)
 * @param utc_month UTC month (1-12)
 * @param utc_day   UTC day (1-31)
 * @return          Timezone result with offsets in minutes
 */
tz_result_t tz_lookup(double lat, double lon, int utc_year, int utc_month, int utc_day);

/**
 * Convenience: get total UTC offset in minutes for a position and date.
 * Equivalent to tz_lookup(...).total_offset_min.
 */
int16_t tz_get_offset_minutes(double lat, double lon, int utc_year, int utc_month, int utc_day);

/**
 * Manual timezone override. When set, tz_lookup() returns this offset
 * instead of computing from GPS position.
 *
 * @param offset_min  UTC offset in minutes (e.g. -480 for UTC-8)
 */
void tz_set_manual_offset(int16_t offset_min);

/**
 * Clear manual override and return to GPS-based automatic timezone.
 */
void tz_set_auto(void);

/**
 * Check if manual timezone override is active.
 */
bool tz_is_manual(void);

#ifdef __cplusplus
}
#endif
