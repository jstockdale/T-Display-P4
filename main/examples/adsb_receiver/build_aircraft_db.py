#!/usr/bin/env python3
"""
build_aircraft_db.py — Convert OpenSky aircraft database CSV to bucket-indexed
binary format for ADS-B Scope firmware.

Usage:
    python3 build_aircraft_db.py aircraft-database-complete-2025-08.csv aircraft.db

File format (version 2):
    Magic:   8 bytes — "ADSBDB02" (version identifier)
    Header:  65536 × 8 bytes = 512 KB
             Each entry: { uint32_t offset; uint32_t count; }
             Indexed by top 16 bits of ICAO24

    Data:    Variable-length records grouped by bucket, sorted by ICAO within bucket.
             Record: icao_lo(1) + acClass(1) + reg\0 + typecode\0 + manufacturer\0 +
                     model\0 + owner\0 + operatorIcao\0

    Lookup:  bucket = icao >> 8
             header[bucket] → { offset into file, record count }
             fseek(offset), fread entire bucket, linear scan matching icao & 0xFF

Filtering:
    - Drop records with no registration AND no typecode
    - Drop surface vehicles and obstacles
    - Keep everything else (including paragliders, ultralights, etc.)

Field limits:
    registration: 10 chars, typecode: 5 chars, manufacturer: 20 chars,
    model: 24 chars, owner: 24 chars, operatorIcao: 4 chars
    Pipes stripped from all values (pipe is the serial delimiter).

Aircraft class (1 byte):
    L = landplane, H = helicopter, A = amphibian, G = gyrocopter,
    T = tilt-rotor, ? = unknown. Derived from icaoAircraftClass first char.

Source: https://opensky-network.org/datasets/#metadata/
"""

import csv
import struct
import sys
import os

# File format version
MAGIC = b'ADSBDB02'
MAGIC_SIZE = 8

# Field length limits
MAX_REG = 10
MAX_TC = 5
MAX_MFR = 20
MAX_MODEL = 24
MAX_OWNER = 24
MAX_OP_ICAO = 4

# Valid aircraft class codes
VALID_CLASSES = set(b'LHAGT')

# Categories to drop
DROP_CATEGORIES = [
    'Surface Vehicle',
    'Cluster Obstacle',
    'Line Obstacle',
    'Point Obstacle',
]

NUM_BUCKETS = 65536
HEADER_SIZE = NUM_BUCKETS * 8  # 512 KB


def clean(s, maxlen):
    """Strip, remove pipes, truncate."""
    s = s.strip().replace('|', ' ')
    if len(s) > maxlen:
        s = s[:maxlen]
    return s


def should_drop(row):
    """Return True if this record should be filtered out."""
    reg = row['registration'].strip()
    tc = row['typecode'].strip()
    if not reg and not tc:
        return True
    cat = row['categoryDescription'].strip()
    for drop in DROP_CATEGORIES:
        if drop in cat:
            return True
    return False


def get_ac_class(row):
    """Extract single-char aircraft class from icaoAircraftClass."""
    ic = row['icaoAircraftClass'].strip()
    if not ic:
        return ord('?')
    ch = ord(ic[0])
    if ch in VALID_CLASSES:
        return ch
    return ord('?')


def build_record(icao_int, row):
    """Build a variable-length binary record for one aircraft."""
    icao_lo = icao_int & 0xFF
    ac_class = get_ac_class(row)

    reg = clean(row['registration'], MAX_REG)
    tc = clean(row['typecode'], MAX_TC)
    mfr = clean(row['manufacturerName'], MAX_MFR)
    model = clean(row['model'], MAX_MODEL)
    owner = clean(row['owner'], MAX_OWNER)
    op_icao = clean(row['operatorIcao'], MAX_OP_ICAO)

    # Pack: icao_lo(1) + acClass(1) + null-terminated strings
    data = struct.pack('BB', icao_lo, ac_class)
    for field in [reg, tc, mfr, model, owner, op_icao]:
        data += field.encode('utf-8', errors='replace') + b'\0'
    return data


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.csv> <output.db>")
        sys.exit(1)

    csv_path = sys.argv[1]
    db_path = sys.argv[2]

    print(f"Reading {csv_path}...")

    # Pass 1: read and bucket all records
    buckets = {}  # bucket_id → [(icao_int, record_bytes), ...]
    total = 0
    kept = 0
    dropped = 0
    class_counts = {}

    with open(csv_path, 'r', encoding='utf-8', errors='replace') as f:
        reader = csv.DictReader(f, quotechar="'")
        for row in reader:
            total += 1
            if should_drop(row):
                dropped += 1
                continue

            try:
                icao_int = int(row['icao24'].strip(), 16)
            except ValueError:
                dropped += 1
                continue

            bucket_id = icao_int >> 8
            record = build_record(icao_int, row)

            # Track class distribution
            ac_class = chr(record[1])
            class_counts[ac_class] = class_counts.get(ac_class, 0) + 1

            if bucket_id not in buckets:
                buckets[bucket_id] = []
            buckets[bucket_id].append((icao_int, record))
            kept += 1

    print(f"  Total: {total:,}  Kept: {kept:,}  Dropped: {dropped:,}")
    print(f"  Non-empty buckets: {len(buckets):,} / {NUM_BUCKETS:,}")
    print(f"  Aircraft classes:")
    for k in sorted(class_counts.keys(), key=lambda x: -class_counts[x]):
        print(f"    {k}: {class_counts[k]:>7,} ({100*class_counts[k]/kept:.1f}%)")

    # Sort each bucket by full ICAO
    for bucket_id in buckets:
        buckets[bucket_id].sort(key=lambda x: x[0])

    # Pass 2: write the file
    print(f"Writing {db_path}...")

    with open(db_path, 'wb') as f:
        # Write magic
        f.write(MAGIC)

        # Write placeholder header
        f.write(b'\0' * HEADER_SIZE)

        # Write data buckets and record their offsets
        header = [(0, 0)] * NUM_BUCKETS

        for bucket_id in range(NUM_BUCKETS):
            if bucket_id not in buckets:
                continue
            records = buckets[bucket_id]
            offset = f.tell()
            count = len(records)
            for icao_int, record in records:
                f.write(record)
            header[bucket_id] = (offset, count)

        # Seek back and write real header (after magic)
        f.seek(MAGIC_SIZE)
        for offset, count in header:
            f.write(struct.pack('<II', offset, count))

    file_size = os.path.getsize(db_path)
    print(f"  File size: {file_size:,} bytes ({file_size/1024/1024:.1f} MB)")
    print(f"  Magic: {MAGIC.decode()}")
    print(f"  Header: {HEADER_SIZE:,} bytes ({HEADER_SIZE/1024:.0f} KB)")
    print(f"  Data: {file_size - HEADER_SIZE - MAGIC_SIZE:,} bytes ({(file_size - HEADER_SIZE - MAGIC_SIZE)/1024/1024:.1f} MB)")

    # Verification
    print("\nVerification — reading back 5 random lookups...")
    import random
    with open(db_path, 'rb') as f:
        magic = f.read(MAGIC_SIZE)
        assert magic == MAGIC, f"Bad magic: {magic}"

        all_icaos = []
        for recs in buckets.values():
            for icao_int, _ in recs:
                all_icaos.append(icao_int)
        samples = random.sample(all_icaos, min(5, len(all_icaos)))

        for icao in samples:
            bucket_id = icao >> 8
            f.seek(MAGIC_SIZE + bucket_id * 8)
            offset, count = struct.unpack('<II', f.read(8))
            f.seek(offset)

            found = False
            for _ in range(count):
                icao_lo, ac_class = struct.unpack('BB', f.read(2))
                fields = []
                for _ in range(6):
                    field = b''
                    while True:
                        ch = f.read(1)
                        if ch == b'\0' or ch == b'':
                            break
                        field += ch
                    fields.append(field.decode('utf-8', errors='replace'))

                if icao_lo == (icao & 0xFF):
                    reg, tc, mfr, model, owner, op = fields
                    print(f"  {icao:06X} [{chr(ac_class)}] {reg:10s} {tc:5s} {mfr[:15]:15s} {model[:20]:20s} {owner[:20]:20s} {op}")
                    found = True
                    break

            if not found:
                print(f"  {icao:06X} -> NOT FOUND (bug!)")

    print(f"\nDone! Copy aircraft.db to /sdcard/data/ on your SD card.")


if __name__ == '__main__':
    main()
