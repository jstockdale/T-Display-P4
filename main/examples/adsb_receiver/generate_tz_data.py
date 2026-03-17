#!/usr/bin/env python3
"""
generate_tz_data.py — Generate timezone lookup data for ESP32 firmware.

Downloads timezone boundary GeoJSON, rasterizes to 0.1° grid, extracts
DST rules from Python's zoneinfo, RLE-compresses each row, and outputs
a C header file (timezone_data.h) for inclusion in firmware.

Requirements:
    pip install geopandas shapely requests

Usage:
    python3 generate_tz_data.py
    # Outputs: timezone_data.h (copy to main/examples/lvgl_9_ui/)

Resolution: 0.1° (~11km at equator)
Typical output size: ~100-150KB compressed
"""

import os
import sys
import json
import struct
import calendar
import requests
import zipfile
import tempfile
from datetime import datetime, timezone, timedelta
from zoneinfo import ZoneInfo
from pathlib import Path

try:
    import geopandas as gpd
    import numpy as np
    from shapely.geometry import Point
except ImportError:
    print("Missing dependencies. Install with:")
    print("  pip install geopandas shapely requests numpy")
    sys.exit(1)

# --- Configuration ---
RESOLUTION = 0.1   # degrees per cell
LAT_CELLS = int(180 / RESOLUTION)   # 1800
LON_CELLS = int(360 / RESOLUTION)   # 3600
REFERENCE_YEAR = 2025  # Year used to detect DST transitions
SHAPEFILE_URL = "https://github.com/evansiroky/timezone-boundary-builder/releases/latest/download/timezones-with-oceans.geojson.zip"
OUTPUT_FILE = "timezone_data.h"


def download_tz_data(cache_dir):
    """Download timezone boundary GeoJSON if not cached."""
    zip_path = os.path.join(cache_dir, "tz_boundaries.zip")
    geojson_path = os.path.join(cache_dir, "combined-with-oceans.json")

    if os.path.exists(geojson_path):
        print(f"Using cached GeoJSON: {geojson_path}")
        return geojson_path

    print(f"Downloading timezone boundaries...")
    print(f"  URL: {SHAPEFILE_URL}")
    resp = requests.get(SHAPEFILE_URL, stream=True)
    resp.raise_for_status()

    total = int(resp.headers.get('content-length', 0))
    downloaded = 0
    with open(zip_path, 'wb') as f:
        for chunk in resp.iter_content(chunk_size=8192):
            f.write(chunk)
            downloaded += len(chunk)
            if total:
                pct = downloaded * 100 // total
                print(f"\r  Downloaded: {downloaded // 1024}KB / {total // 1024}KB ({pct}%)", end='')
    print()

    print("Extracting...")
    with zipfile.ZipFile(zip_path) as zf:
        # Find the GeoJSON file inside the zip
        geojson_names = [n for n in zf.namelist() if n.endswith('.json') or n.endswith('.geojson')]
        if not geojson_names:
            print(f"  ZIP contents: {zf.namelist()}")
            raise RuntimeError("No GeoJSON found in downloaded archive")
        extracted = zf.extract(geojson_names[0], cache_dir)
        if extracted != geojson_path:
            os.rename(extracted, geojson_path)

    return geojson_path


def get_tz_info(tz_name):
    """Extract standard offset and DST transitions for a timezone."""
    try:
        tz = ZoneInfo(tz_name)
    except Exception:
        # Fall back to parsing Etc/GMT zones
        if tz_name.startswith("Etc/GMT"):
            sign_part = tz_name[7:]
            if sign_part == "" or sign_part == "0":
                return {'std_offset_min': 0, 'has_dst': False}
            # Etc zones have INVERTED signs (Etc/GMT+5 = UTC-5)
            offset_h = -int(sign_part)
            return {'std_offset_min': offset_h * 60, 'has_dst': False}
        print(f"  Warning: unknown timezone '{tz_name}', defaulting to UTC")
        return {'std_offset_min': 0, 'has_dst': False}

    # Get winter and summer offsets to detect DST
    jan = datetime(REFERENCE_YEAR, 1, 15, 12, 0, tzinfo=timezone.utc).astimezone(tz)
    jul = datetime(REFERENCE_YEAR, 7, 15, 12, 0, tzinfo=timezone.utc).astimezone(tz)

    jan_off = jan.utcoffset().total_seconds() / 60
    jul_off = jul.utcoffset().total_seconds() / 60

    has_dst = abs(jan_off - jul_off) > 0

    if not has_dst:
        return {'std_offset_min': int(jan_off), 'has_dst': False}

    # Determine which is standard time (shorter offset = standard)
    if abs(jan_off) <= abs(jul_off):
        # Northern hemisphere DST pattern (summer offset is larger)
        std_offset = int(jan_off)
        dst_offset = int(jul_off - jan_off)
        southern = False
    else:
        # Southern hemisphere (winter=Jul has standard time)
        std_offset = int(jul_off)
        dst_offset = int(jan_off - jul_off)
        southern = True

    # Find transition dates by scanning each day
    transitions = find_transitions(tz, REFERENCE_YEAR)

    return {
        'std_offset_min': std_offset,
        'has_dst': True,
        'dst_offset_min': dst_offset,
        'southern': southern,
        'transitions': transitions,
    }


def find_transitions(tz, year):
    """Find DST transition dates by scanning the year day-by-day."""
    transitions = []
    prev_off = datetime(year, 1, 1, 12, 0, tzinfo=timezone.utc).astimezone(tz).utcoffset()

    for month in range(1, 13):
        for day in range(1, 32):
            try:
                dt = datetime(year, month, day, 12, 0, tzinfo=timezone.utc)
                off = dt.astimezone(tz).utcoffset()
                if off != prev_off:
                    # Transition happened on this day
                    rule = date_to_weekday_rule(year, month, day)
                    transitions.append({
                        'month': month,
                        'week': rule['week'],
                        'dow': rule['dow'],
                        'direction': 'spring' if off > prev_off else 'fall',
                    })
                    prev_off = off
            except ValueError:
                continue

    return transitions


def date_to_weekday_rule(year, month, day):
    """Convert a date to 'Nth weekday of month' rule."""
    dt = datetime(year, month, day)
    dow = dt.isoweekday() % 7  # Sunday=0, Monday=1, ..., Saturday=6

    # Which occurrence of this weekday in the month
    week_num = (day - 1) // 7 + 1

    # Check if this is the last occurrence
    days_in_month = calendar.monthrange(year, month)[1]
    if day + 7 > days_in_month:
        week_num = 5  # Convention: 5 = "last"

    return {'week': week_num, 'dow': dow}


def build_dst_rule_table(tz_infos):
    """Build a deduplicated table of DST rules. Returns (rules_list, tz_to_rule_id)."""
    rules = [None]  # Index 0 = no DST
    rule_map = {}   # (start_month, start_week, start_dow, end_month, end_week, end_dow, dst_off, southern) → rule_id
    tz_rule_ids = {}

    for tz_name, info in tz_infos.items():
        if not info['has_dst'] or not info.get('transitions'):
            tz_rule_ids[tz_name] = 0
            continue

        trans = info['transitions']
        if len(trans) < 2:
            tz_rule_ids[tz_name] = 0
            continue

        # Spring forward and fall back
        spring = [t for t in trans if t['direction'] == 'spring']
        fall = [t for t in trans if t['direction'] == 'fall']

        if not spring or not fall:
            tz_rule_ids[tz_name] = 0
            continue

        s = spring[0]
        f = fall[0]

        southern = 1 if info.get('southern', False) else 0
        dst_off_qh = info['dst_offset_min'] // 15

        key = (s['month'], s['week'], s['dow'],
               f['month'], f['week'], f['dow'],
               dst_off_qh, southern)

        if key not in rule_map:
            rule_id = len(rules)
            rule_map[key] = rule_id
            rules.append({
                'start_month': s['month'],
                'start_week': s['week'],
                'start_dow': s['dow'],
                'end_month': f['month'],
                'end_week': f['week'],
                'end_dow': f['dow'],
                'dst_offset_qh': dst_off_qh,
                'southern': southern,
            })

        tz_rule_ids[tz_name] = rule_map[key]

    return rules, tz_rule_ids


def build_region_table(tz_infos, tz_rule_ids):
    """Build deduplicated region table. Returns (regions_list, tz_to_region_id)."""
    regions = []
    region_map = {}  # (std_offset_qh, rule_id) → region_id
    tz_region_ids = {}

    for tz_name, info in tz_infos.items():
        std_off_qh = info['std_offset_min'] // 15
        rule_id = tz_rule_ids.get(tz_name, 0)
        key = (std_off_qh, rule_id)

        if key not in region_map:
            region_id = len(regions)
            region_map[key] = region_id
            regions.append({
                'base_offset_qh': std_off_qh,
                'dst_rule_id': rule_id,
            })

        tz_region_ids[tz_name] = region_map[key]

    return regions, tz_region_ids


def rasterize_grid(gdf, tz_region_ids, default_region_id=0):
    """Rasterize timezone polygons to a 0.1° grid. Returns 2D numpy array."""
    print(f"Rasterizing {LAT_CELLS}x{LON_CELLS} grid ({LAT_CELLS * LON_CELLS:,} cells)...")
    print("  This may take several minutes...")

    grid = np.full((LAT_CELLS, LON_CELLS), default_region_id, dtype=np.uint8)

    # Build map from timezone name to region_id
    tz_to_rid = {}
    for _, row in gdf.iterrows():
        tz_name = row['tzid']
        tz_to_rid[tz_name] = tz_region_ids.get(tz_name, default_region_id)

    # Generate grid center points in batches (by row) for memory efficiency
    total_rows = LAT_CELLS
    for lat_idx in range(LAT_CELLS):
        if lat_idx % 100 == 0:
            pct = lat_idx * 100 // LAT_CELLS
            print(f"\r  Row {lat_idx}/{LAT_CELLS} ({pct}%)", end='')

        lat = -90.0 + (lat_idx + 0.5) * RESOLUTION

        # Generate points for this latitude row
        lons = np.arange(-180.0 + RESOLUTION / 2, 180.0, RESOLUTION)
        points = [Point(lon, lat) for lon in lons]
        points_gdf = gpd.GeoDataFrame(
            {'lon_idx': range(len(points))},
            geometry=points,
            crs=gdf.crs
        )

        # Spatial join
        joined = gpd.sjoin(points_gdf, gdf, how='left', predicate='within')

        for _, match in joined.iterrows():
            lon_idx = int(match['lon_idx'])
            tz_name = match.get('tzid', None)
            if tz_name and tz_name in tz_to_rid:
                grid[lat_idx, lon_idx] = tz_to_rid[tz_name]

    print(f"\r  Row {LAT_CELLS}/{LAT_CELLS} (100%)  ")
    return grid


def rle_compress(grid):
    """RLE compress each row. Returns (row_offsets, rle_data)."""
    print("RLE compressing...")
    row_offsets = []
    rle_bytes = bytearray()

    for lat_idx in range(LAT_CELLS):
        row_offsets.append(len(rle_bytes))
        row = grid[lat_idx]

        # RLE encode: (value, count) pairs
        i = 0
        while i < LON_CELLS:
            val = row[i]
            count = 1
            while i + count < LON_CELLS and row[i + count] == val and count < 65535:
                count += 1
            # Pack: 1 byte value + 2 bytes count (little-endian)
            rle_bytes.append(val)
            rle_bytes.extend(struct.pack('<H', count))
            i += count

    total_runs = len(rle_bytes) // 3
    print(f"  {total_runs:,} runs, {len(rle_bytes):,} bytes ({len(rle_bytes) / 1024:.1f} KB)")
    print(f"  Compression ratio: {grid.nbytes / len(rle_bytes):.1f}x")

    return row_offsets, bytes(rle_bytes)


def generate_c_header(regions, dst_rules, row_offsets, rle_data):
    """Generate the C header file."""
    print(f"Generating {OUTPUT_FILE}...")

    lines = []
    lines.append("// Auto-generated by generate_tz_data.py — do not edit manually")
    lines.append(f"// Resolution: {RESOLUTION}° ({RESOLUTION * 111:.0f}km at equator)")
    lines.append(f"// Reference year for DST rules: {REFERENCE_YEAR}")
    lines.append(f"// Regions: {len(regions)}, DST rules: {len(dst_rules)}")
    lines.append(f"// RLE data: {len(rle_data)} bytes ({len(rle_data) / 1024:.1f} KB)")
    lines.append(f"// Grid: {LAT_CELLS} rows x {LON_CELLS} columns")
    lines.append("")
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define TZ_RESOLUTION    {RESOLUTION}")
    lines.append(f"#define TZ_LAT_CELLS     {LAT_CELLS}")
    lines.append(f"#define TZ_LON_CELLS     {LON_CELLS}")
    lines.append(f"#define TZ_NUM_REGIONS   {len(regions)}")
    lines.append(f"#define TZ_NUM_DST_RULES {len(dst_rules)}")
    lines.append("")

    # DST rule struct and table
    lines.append("typedef struct {")
    lines.append("    uint8_t start_month;     // 1-12")
    lines.append("    uint8_t start_week;      // 1-4=Nth, 5=last")
    lines.append("    uint8_t start_dow;       // 0=Sun, 1=Mon, ..., 6=Sat")
    lines.append("    uint8_t end_month;")
    lines.append("    uint8_t end_week;")
    lines.append("    uint8_t end_dow;")
    lines.append("    int8_t  dst_offset_qh;   // DST offset in quarter-hours (usually +4)")
    lines.append("    uint8_t southern;         // 1 if start_month > end_month")
    lines.append("} tz_dst_rule_t;")
    lines.append("")

    lines.append("typedef struct {")
    lines.append("    int8_t  base_offset_qh;  // Standard UTC offset in quarter-hours")
    lines.append("    uint8_t dst_rule_id;      // Index into tz_dst_rules (0=no DST)")
    lines.append("} tz_region_t;")
    lines.append("")

    # DST rules table
    lines.append(f"static const tz_dst_rule_t tz_dst_rules[{len(dst_rules)}] = {{")
    for i, rule in enumerate(dst_rules):
        if rule is None:
            lines.append(f"    {{0, 0, 0, 0, 0, 0, 0, 0}},  // 0: no DST")
        else:
            lines.append(f"    {{{rule['start_month']}, {rule['start_week']}, {rule['start_dow']}, "
                         f"{rule['end_month']}, {rule['end_week']}, {rule['end_dow']}, "
                         f"{rule['dst_offset_qh']}, {rule['southern']}}},  // {i}")
    lines.append("};")
    lines.append("")

    # Region table
    lines.append(f"static const tz_region_t tz_regions[{len(regions)}] = {{")
    for i, region in enumerate(regions):
        off_h = region['base_offset_qh'] / 4.0
        sign = '+' if off_h >= 0 else ''
        lines.append(f"    {{{region['base_offset_qh']}, {region['dst_rule_id']}}},  "
                     f"// {i}: UTC{sign}{off_h:g}" +
                     (f" +DST rule {region['dst_rule_id']}" if region['dst_rule_id'] else ""))
    lines.append("};")
    lines.append("")

    # Row offset table
    lines.append(f"static const uint32_t tz_row_offsets[{LAT_CELLS}] = {{")
    for i in range(0, LAT_CELLS, 10):
        chunk = row_offsets[i:i+10]
        lines.append("    " + ", ".join(str(x) for x in chunk) + ",")
    lines.append("};")
    lines.append("")

    # RLE data — output as hex bytes, 20 per line
    lines.append(f"static const uint8_t tz_rle_data[{len(rle_data)}] = {{")
    for i in range(0, len(rle_data), 20):
        chunk = rle_data[i:i+20]
        hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
        lines.append(f"    {hex_str},")
    lines.append("};")
    lines.append("")

    with open(OUTPUT_FILE, 'w') as f:
        f.write('\n'.join(lines))

    file_size = os.path.getsize(OUTPUT_FILE)
    print(f"  Written: {OUTPUT_FILE} ({file_size / 1024:.1f} KB)")


def main():
    print("=== Timezone Data Generator for ESP32 Firmware ===")
    print(f"Grid: {LON_CELLS}x{LAT_CELLS} ({RESOLUTION}° resolution)")
    print()

    # Use a cache directory for downloaded data
    cache_dir = os.path.join(tempfile.gettempdir(), "tz_gen_cache")
    os.makedirs(cache_dir, exist_ok=True)

    # Step 1: Download timezone boundaries
    geojson_path = download_tz_data(cache_dir)

    # Step 2: Load GeoJSON
    print("Loading timezone boundaries...")
    gdf = gpd.read_file(geojson_path)
    tz_names = sorted(gdf['tzid'].unique())
    print(f"  {len(tz_names)} timezone zones loaded")

    # Step 3: Extract timezone info (offsets + DST)
    print("Extracting timezone offsets and DST rules...")
    tz_infos = {}
    for i, tz_name in enumerate(tz_names):
        if i % 50 == 0:
            print(f"\r  Processing {i}/{len(tz_names)}...", end='')
        tz_infos[tz_name] = get_tz_info(tz_name)
    print(f"\r  Processed {len(tz_names)} timezones     ")

    # Count DST zones
    dst_count = sum(1 for v in tz_infos.values() if v['has_dst'])
    print(f"  {dst_count} zones with DST, {len(tz_names) - dst_count} without")

    # Step 4: Build DST rule table
    dst_rules, tz_rule_ids = build_dst_rule_table(tz_infos)
    print(f"  {len(dst_rules)} unique DST rules (including 'no DST')")

    # Step 5: Build region table
    regions, tz_region_ids = build_region_table(tz_infos, tz_rule_ids)
    print(f"  {len(regions)} unique regions")

    if len(regions) > 255:
        print(f"  WARNING: {len(regions)} regions exceeds uint8_t max (255)")
        print(f"  Consider reducing resolution or merging similar regions")
        sys.exit(1)

    # Step 6: Rasterize to grid
    grid = rasterize_grid(gdf, tz_region_ids)

    # Step 7: RLE compress
    row_offsets, rle_data = rle_compress(grid)

    # Step 8: Generate C header
    generate_c_header(regions, dst_rules, row_offsets, rle_data)

    print()
    print("Done! Copy timezone_data.h to your firmware source directory.")
    print("Include it in tz_lookup.c to enable GPS-based timezone detection.")


if __name__ == '__main__':
    main()
