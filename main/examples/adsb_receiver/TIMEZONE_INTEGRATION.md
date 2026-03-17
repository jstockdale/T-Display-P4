# Timezone Lookup Integration Guide

## Files to add to your project

1. **`tz_lookup.h`** — Public API header
2. **`tz_lookup.c`** — Lookup implementation
3. **`timezone_data.h`** — Generated data (run `generate_tz_data.py` first)

Place all three in `main/examples/lvgl_9_ui/` alongside your other source files.

## Step 0: Generate timezone_data.h

On your Mac (needs network access):

```bash
pip install geopandas shapely requests numpy
python3 generate_tz_data.py
# Takes 5-15 minutes (rasterizes 6.48M grid cells)
# Outputs: timezone_data.h (~100-150KB)
```

Copy `timezone_data.h` to your firmware source directory.

## Step 1: Add to CMakeLists.txt

The `SRC_DIRS` glob should pick up `tz_lookup.c` automatically since it's
in the same directory. No CMakeLists.txt changes needed.

## Step 2: Add include to main.cpp

```cpp
#include "tz_lookup.h"
```

## Step 3: Replace hardcoded UTC+8 in GPS task

In `device_gps_task`, where the RTC is set, replace the hardcoded `+ 8 * 3600`
with a dynamic lookup:

```cpp
// Old:
time_t epoch_local = gps_epoch + 8 * 3600;

// New:
struct tm tm_utc;
gmtime_r(&gps_epoch, &tm_utc);
int16_t tz_off = tz_get_offset_minutes(lat, lon,
    tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
time_t epoch_local = gps_epoch + (tz_off * 60);
```

## Step 4: Replace hardcoded UTC+8 in display time

In `Save_Real_Time` and anywhere that converts UTC to local with `+ 8`:

```cpp
// Old:
.hour = static_cast<uint8_t>((time.hour + 8 + 24) % 24),

// New: use tz_get_offset_minutes() with current GPS position
receiver_pos_t rx = adsb_get_receiver_pos();
int16_t tz_off = 0;
if (rx.fix_valid) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm tm_utc;
    gmtime_r(&tv.tv_sec, &tm_utc);
    tz_off = tz_get_offset_minutes(rx.lat, rx.lon,
        tm_utc.tm_year + 1900, tm_utc.tm_mon + 1, tm_utc.tm_mday);
}
int offset_hours = tz_off / 60;
// Then use offset_hours instead of hardcoded 8
```

## Step 5: Log timezone on first fix

In the GPS task, after the first position fix, log the detected timezone:

```cpp
static bool tz_logged = false;
if (!tz_logged && has_pos) {
    tz_result_t tz = tz_lookup(lat, lon, utc_year, utc_mon, utc_day);
    serial_console_print("[GNSS] Timezone: UTC%+d%s%s\n",
        tz.total_offset_min / 60,
        (abs(tz.total_offset_min) % 60) ? ":30" : "",
        tz.dst_active ? " (DST)" : "");
    tz_logged = true;
}
```

## Serial console commands

```
timezone              show current timezone info
timezone auto         use GPS-based automatic timezone
timezone UTC-8        set manual offset (PST)
timezone UTC+5:30     set manual offset (IST)
timezone UTC+12:45    set manual offset (Chatham Islands)
tz                    alias for timezone
```

## How it works

1. GPS fix provides lat/lon
2. `tz_lookup()` maps lat/lon to a 0.1° RLE-compressed grid → region_id
3. Region table gives standard UTC offset + DST rule index
4. DST rule is checked against the current UTC date
5. Total offset = standard + DST (if active)

## Memory usage

- Flash: ~100-150KB (timezone_data.h compiled into .rodata)
- RAM: 6 bytes for manual override state
- Stack: ~100 bytes during lookup

## Limitations

- DST rules are based on a reference year (2025). If a country changes
  its DST rules, regenerate timezone_data.h.
- 0.1° resolution (~11km) may misidentify timezone in border towns.
  Users in those areas can use the manual override.
- Does not handle historical timezone changes (always uses current rules).
