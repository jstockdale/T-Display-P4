#!/usr/bin/env python3
"""
SD Card Forensic Analysis for ADS-B Scope
Analyzes SD cards for FAT32 corruption, dirty flags, log file integrity,
and bit-flip patterns. Works in two modes:

Live mode (mounted card — quick checks):
    python3 sd_forensics.py /Volumes/SD32G
    python3 sd_forensics.py /Volumes/SD32G --dump-logs
    python3 sd_forensics.py --find                      # auto-detect
    sudo python3 sd_forensics.py /Volumes/SD32G         # + BPB/FAT via raw device

Raw image mode (full forensic analysis):
    diskutil unmountDisk /dev/disk4
    sudo dd if=/dev/rdisk4 of=~/sd.img bs=4096 conv=noerror,sync status=progress
    python3 sd_forensics.py ~/sd.img
    python3 sd_forensics.py ~/sd.img --dump-logs
"""

import sys
import os
import struct
import hashlib
import argparse
from collections import Counter


# ── FAT32 BPB parsing ────────────────────────────────────────────────────────

def parse_bpb(data):
    """Parse FAT32 BIOS Parameter Block from first 512 bytes."""
    if len(data) < 512:
        return None

    bpb = {}
    bpb['jump'] = data[0:3]
    bpb['oem_name'] = data[3:11].decode('ascii', errors='replace').strip()
    bpb['bytes_per_sector'] = struct.unpack_from('<H', data, 11)[0]
    bpb['sectors_per_cluster'] = data[13]
    bpb['reserved_sectors'] = struct.unpack_from('<H', data, 14)[0]
    bpb['num_fats'] = data[16]
    bpb['total_sectors_16'] = struct.unpack_from('<H', data, 19)[0]
    bpb['media_type'] = data[21]
    bpb['fat_size_16'] = struct.unpack_from('<H', data, 22)[0]
    bpb['total_sectors_32'] = struct.unpack_from('<I', data, 32)[0]

    # FAT32-specific
    bpb['fat_size_32'] = struct.unpack_from('<I', data, 36)[0]
    bpb['ext_flags'] = struct.unpack_from('<H', data, 40)[0]
    bpb['fs_version'] = struct.unpack_from('<H', data, 42)[0]
    bpb['root_cluster'] = struct.unpack_from('<I', data, 44)[0]
    bpb['fs_info_sector'] = struct.unpack_from('<H', data, 48)[0]
    bpb['backup_boot_sector'] = struct.unpack_from('<H', data, 50)[0]

    # Dirty bit is in FAT32 extended boot record
    # Byte 65 (0x41) of BPB: bit 0 = dirty, bit 1 = I/O error
    bpb['dirty_byte'] = data[0x41]
    bpb['dirty'] = not (data[0x41] & 0x01)  # bit 0 CLEAR = dirty
    bpb['io_error'] = not (data[0x41] & 0x02)  # bit 1 CLEAR = I/O error

    bpb['volume_label'] = data[71:82].decode('ascii', errors='replace').strip()
    bpb['fs_type'] = data[82:90].decode('ascii', errors='replace').strip()

    # Derived values
    fat_size = bpb['fat_size_32'] if bpb['fat_size_32'] else bpb['fat_size_16']
    bpb['fat_size'] = fat_size
    bps = bpb['bytes_per_sector']
    bpb['fat1_offset'] = bpb['reserved_sectors'] * bps
    bpb['fat2_offset'] = (bpb['reserved_sectors'] + fat_size) * bps
    bpb['fat_bytes'] = fat_size * bps
    total = bpb['total_sectors_32'] if bpb['total_sectors_32'] else bpb['total_sectors_16']
    bpb['total_sectors'] = total
    data_start_sector = bpb['reserved_sectors'] + bpb['num_fats'] * fat_size
    bpb['data_start_sector'] = data_start_sector
    bpb['data_start_offset'] = data_start_sector * bps
    bpb['total_data_clusters'] = (total - data_start_sector) // bpb['sectors_per_cluster']
    bpb['cluster_size'] = bpb['sectors_per_cluster'] * bps

    return bpb


def parse_fsinfo(data, bpb):
    """Parse FSInfo sector."""
    offset = bpb['fs_info_sector'] * bpb['bytes_per_sector']
    if len(data) <= offset + 512:
        return None

    fsinfo = data[offset:offset + 512]
    info = {}
    info['lead_sig'] = struct.unpack_from('<I', fsinfo, 0)[0]
    info['struc_sig'] = struct.unpack_from('<I', fsinfo, 484)[0]
    info['free_count'] = struct.unpack_from('<I', fsinfo, 488)[0]
    info['next_free'] = struct.unpack_from('<I', fsinfo, 492)[0]
    info['trail_sig'] = struct.unpack_from('<I', fsinfo, 508)[0]

    info['valid'] = (info['lead_sig'] == 0x41615252 and
                     info['struc_sig'] == 0x61417272 and
                     info['trail_sig'] == 0xAA550000)
    return info


# ── FAT table analysis ────────────────────────────────────────────────────────

def read_fat(data, bpb, fat_num=0):
    """Read FAT entries as a list of 32-bit values."""
    if fat_num == 0:
        offset = bpb['fat1_offset']
    else:
        offset = bpb['fat2_offset']

    fat_bytes = bpb['fat_bytes']
    if len(data) < offset + fat_bytes:
        print(f"  WARNING: image too small for FAT{fat_num + 1} "
              f"(need {offset + fat_bytes}, have {len(data)})")
        fat_bytes = min(fat_bytes, len(data) - offset)

    num_entries = fat_bytes // 4
    entries = []
    for i in range(num_entries):
        val = struct.unpack_from('<I', data, offset + i * 4)[0] & 0x0FFFFFFF
        entries.append(val)
    return entries


def compare_fats(data, bpb):
    """Compare FAT1 and FAT2, return list of differing entries."""
    fat1_off = bpb['fat1_offset']
    fat2_off = bpb['fat2_offset']
    fat_bytes = bpb['fat_bytes']

    if len(data) < max(fat1_off, fat2_off) + fat_bytes:
        return None, None, None

    fat1_raw = data[fat1_off:fat1_off + fat_bytes]
    fat2_raw = data[fat2_off:fat2_off + fat_bytes]

    if fat1_raw == fat2_raw:
        return [], hashlib.md5(fat1_raw).hexdigest(), hashlib.md5(fat2_raw).hexdigest()

    diffs = []
    for i in range(fat_bytes // 4):
        v1 = struct.unpack_from('<I', fat1_raw, i * 4)[0] & 0x0FFFFFFF
        v2 = struct.unpack_from('<I', fat2_raw, i * 4)[0] & 0x0FFFFFFF
        if v1 != v2:
            diffs.append((i, v1, v2))

    return diffs, hashlib.md5(fat1_raw).hexdigest(), hashlib.md5(fat2_raw).hexdigest()


def analyze_fat(fat_entries, bpb):
    """Analyze FAT for free clusters, chains, lost clusters, etc."""
    total = bpb['total_data_clusters'] + 2  # entries 0,1 are reserved
    if len(fat_entries) < total:
        total = len(fat_entries)

    free = 0
    bad = 0
    used = 0
    eoc = 0  # end of chain
    reserved = 0

    for i in range(2, total):
        val = fat_entries[i]
        if val == 0:
            free += 1
        elif val == 0x0FFFFFF7:
            bad += 1
        elif val >= 0x0FFFFFF8:
            eoc += 1
            used += 1
        elif val >= 0x0FFFFFF0:
            reserved += 1
        else:
            used += 1

    # Find chain heads (used clusters not pointed to by any other entry)
    pointed_to = set()
    for i in range(2, total):
        val = fat_entries[i]
        if 2 <= val < 0x0FFFFFF0:
            pointed_to.add(val)

    chain_heads = set()
    for i in range(2, total):
        if fat_entries[i] != 0 and i not in pointed_to:
            chain_heads.add(i)

    return {
        'total_entries': total - 2,
        'free': free,
        'used': used,
        'bad': bad,
        'reserved': reserved,
        'eoc_count': eoc,  # number of end-of-chain markers = number of file/dir endings
        'chain_heads': len(chain_heads),
    }


# ── Directory parsing ─────────────────────────────────────────────────────────

def cluster_to_offset(cluster, bpb):
    """Convert cluster number to byte offset in image."""
    return bpb['data_start_offset'] + (cluster - 2) * bpb['cluster_size']


def follow_chain(fat, start_cluster):
    """Follow a FAT chain from start_cluster, return list of clusters."""
    chain = []
    cluster = start_cluster
    seen = set()
    while 2 <= cluster < 0x0FFFFFF0:
        if cluster in seen:
            chain.append(('LOOP', cluster))
            break
        seen.add(cluster)
        chain.append(cluster)
        if cluster < len(fat):
            cluster = fat[cluster]
        else:
            break
    return chain


def read_cluster_chain(data, fat, start_cluster, bpb):
    """Read all data from a cluster chain."""
    chain = follow_chain(fat, start_cluster)
    result = bytearray()
    for c in chain:
        if isinstance(c, tuple):
            break  # loop detected
        off = cluster_to_offset(c, bpb)
        end = off + bpb['cluster_size']
        if end <= len(data):
            result.extend(data[off:end])
    return bytes(result)


def parse_directory(dir_data):
    """Parse FAT32 directory entries, return list of file info dicts."""
    entries = []
    lfn_parts = []
    i = 0
    while i + 32 <= len(dir_data):
        entry = dir_data[i:i + 32]
        i += 32

        if entry[0] == 0x00:
            break  # end of directory
        if entry[0] == 0xE5:
            continue  # deleted entry

        attr = entry[11]

        # Long filename entry
        if attr == 0x0F:
            seq = entry[0] & 0x3F
            name_part = b''
            for off in [1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30]:
                if off + 1 < len(entry):
                    c = struct.unpack_from('<H', entry, off)[0]
                    if c == 0 or c == 0xFFFF:
                        break
                    name_part += chr(c).encode('utf-8', errors='replace')
            lfn_parts.append((seq, name_part.decode('utf-8', errors='replace')))
            continue

        # Short name entry
        short_name = entry[0:8].decode('ascii', errors='replace').strip()
        short_ext = entry[8:11].decode('ascii', errors='replace').strip()
        if short_ext:
            short_name = f"{short_name}.{short_ext}"

        # Reconstruct LFN
        long_name = None
        if lfn_parts:
            lfn_parts.sort(key=lambda x: x[0])
            long_name = ''.join(p[1] for p in lfn_parts)
            lfn_parts = []
        else:
            lfn_parts = []

        cluster_hi = struct.unpack_from('<H', entry, 20)[0]
        cluster_lo = struct.unpack_from('<H', entry, 26)[0]
        cluster = (cluster_hi << 16) | cluster_lo
        size = struct.unpack_from('<I', entry, 28)[0]

        # Dates
        ctime_raw = struct.unpack_from('<H', entry, 14)[0]
        cdate_raw = struct.unpack_from('<H', entry, 16)[0]
        mtime_raw = struct.unpack_from('<H', entry, 22)[0]
        mdate_raw = struct.unpack_from('<H', entry, 24)[0]

        def decode_date(d):
            if d == 0:
                return "0000-00-00"
            return f"{1980 + (d >> 9):04d}-{(d >> 5) & 0xF:02d}-{d & 0x1F:02d}"

        def decode_time(t):
            return f"{(t >> 11) & 0x1F:02d}:{(t >> 5) & 0x3F:02d}:{(t & 0x1F) * 2:02d}"

        entries.append({
            'name': long_name or short_name,
            'short_name': short_name,
            'attr': attr,
            'is_dir': bool(attr & 0x10),
            'cluster': cluster,
            'size': size,
            'created': f"{decode_date(cdate_raw)} {decode_time(ctime_raw)}",
            'modified': f"{decode_date(mdate_raw)} {decode_time(mtime_raw)}",
        })

    return entries


# ── Log file analysis ─────────────────────────────────────────────────────────

def analyze_log_file(file_data, filename):
    """Analyze an ADS-B or Meshy CSV log file for integrity."""
    info = {
        'filename': filename,
        'raw_size': len(file_data),
    }

    # Strip trailing nulls (unfilled cluster space)
    text = file_data.rstrip(b'\x00')
    info['data_size'] = len(text)

    if len(text) == 0:
        info['status'] = 'EMPTY'
        return info

    try:
        decoded = text.decode('utf-8', errors='replace')
    except Exception:
        info['status'] = 'BINARY_GARBAGE'
        return info

    lines = decoded.split('\n')
    info['total_lines'] = len(lines)

    # Check header
    if lines[0].startswith('timestamp_utc,'):
        info['has_header'] = True
        info['header'] = lines[0][:80]
    else:
        info['has_header'] = False

    # Check last line
    last_nonempty = ''
    for line in reversed(lines):
        if line.strip():
            last_nonempty = line
            break
    info['last_line'] = last_nonempty[:120]

    # Check for truncated final line (no newline at end)
    if text and text[-1:] != b'\n':
        info['truncated_end'] = True
    else:
        info['truncated_end'] = False

    # Count lines with expected CSV field count
    expected_fields = None
    if info.get('has_header'):
        expected_fields = len(lines[0].split(','))
    good_lines = 0
    bad_lines = 0
    for line in lines[1:]:
        if not line.strip():
            continue
        fields = len(line.split(','))
        if expected_fields and fields == expected_fields:
            good_lines += 1
        else:
            bad_lines += 1

    info['good_lines'] = good_lines
    info['bad_lines'] = bad_lines
    info['status'] = 'OK' if bad_lines == 0 and not info['truncated_end'] else 'ISSUES'

    # Check for null bytes in middle of data
    null_count = text.count(b'\x00')
    info['embedded_nulls'] = null_count

    # Check for bit-flip patterns
    # Look for non-ASCII bytes in what should be ASCII CSV
    non_ascii = sum(1 for b in text if b > 127)
    info['non_ascii_bytes'] = non_ascii

    return info


# ── Bit-flip analysis ─────────────────────────────────────────────────────────

def find_bit_flips(data, offset, length, expected_pattern=None):
    """Look for single-bit errors in a region."""
    flips = []
    region = data[offset:offset + length]
    for i, b in enumerate(region):
        if expected_pattern and i < len(expected_pattern):
            diff = b ^ expected_pattern[i]
            if diff and bin(diff).count('1') == 1:
                flips.append({
                    'offset': offset + i,
                    'got': b,
                    'expected': expected_pattern[i],
                    'bit': diff.bit_length() - 1,
                })
    return flips


# ── Live filesystem analysis (mounted volume) ────────────────────────────────

def find_sd_volumes():
    """Find mounted SD card volumes on macOS."""
    volumes = []
    if not os.path.isdir('/Volumes'):
        return volumes
    for name in os.listdir('/Volumes'):
        vol = os.path.join('/Volumes', name)
        if not os.path.isdir(vol):
            continue
        # Check for our signature files
        has_logs = os.path.isdir(os.path.join(vol, 'logs'))
        has_config = os.path.isfile(os.path.join(vol, 'sd_config.txt'))
        has_music = os.path.isdir(os.path.join(vol, 'music'))
        if has_logs or has_config:
            volumes.append((vol, name, has_logs, has_config, has_music))
    return volumes


def get_raw_device(volume_path):
    """Try to find the raw block device for a mounted volume (macOS)."""
    import subprocess
    try:
        result = subprocess.run(['diskutil', 'info', volume_path],
                                capture_output=True, text=True, timeout=5)
        for line in result.stdout.split('\n'):
            if 'Device Node' in line:
                dev = line.split(':')[-1].strip()
                # Convert /dev/diskNsM to /dev/rdiskNsM for raw access
                raw = dev.replace('/dev/disk', '/dev/rdisk')
                return dev, raw
    except Exception:
        pass
    return None, None


def live_analysis(vol_path, args):
    """Analyze a mounted SD card volume via filesystem APIs."""
    print(f"{'=' * 60}")
    print(f"  ADS-B Scope – SD Card Analysis (live)")
    print(f"{'=' * 60}")
    print(f"  Volume:  {vol_path}")
    print(f"  Mode:    live filesystem (read-only)")
    print()

    # ── Try raw device for BPB/FAT analysis ───────────────────────────────
    dev, raw_dev = get_raw_device(vol_path)
    raw_header = None
    if raw_dev and os.path.exists(raw_dev):
        print(f"  Block device: {dev}")
        print(f"  Raw device:   {raw_dev}")
        try:
            with open(raw_dev, 'rb') as f:
                # Read BPB first to learn FAT layout
                bpb_data = f.read(512)
                bpb_probe = parse_bpb(bpb_data)
                if bpb_probe and bpb_probe['num_fats'] >= 2:
                    # Read enough for BPB + FSInfo + both FATs
                    need = bpb_probe['fat2_offset'] + bpb_probe['fat_bytes'] + 4096
                    f.seek(0)
                    raw_header = f.read(need)
                    print(f"  Read {len(raw_header):,} bytes from raw device "
                          f"(covers both FATs)")
                else:
                    f.seek(0)
                    raw_header = f.read(1024 * 1024)
                    print(f"  Read {len(raw_header):,} bytes from raw device")
        except PermissionError:
            print(f"  NOTE: Cannot read raw device (try: sudo python3 {sys.argv[0]} ...)")
            print(f"        Raw analysis (BPB, dirty flag, FAT comparison) will be skipped.")
            raw_header = None
        except Exception as e:
            print(f"  NOTE: Raw device read failed: {e}")
            raw_header = None
    else:
        print(f"  NOTE: Could not find raw block device — skipping BPB/FAT analysis")
        print(f"        For full analysis, image the card or run with sudo")
    print()

    if raw_header:
        bpb = parse_bpb(raw_header)
        if bpb:
            print(f"{'─' * 60}")
            print("  FAT32 Boot Parameter Block")
            print(f"{'─' * 60}")
            print(f"  Volume Label:      {bpb['volume_label']}")
            print(f"  FS Type:           {bpb['fs_type']}")
            print(f"  Cluster Size:      {bpb['cluster_size']:,} bytes")
            print(f"  Total Sectors:     {bpb['total_sectors']:,}")
            print()

            dirty_byte = bpb['dirty_byte']
            print(f"  Dirty Byte (0x41): 0x{dirty_byte:02X} (binary: {dirty_byte:08b})")
            if bpb['dirty']:
                print(f"  *** DIRTY FLAG SET *** – filesystem was not cleanly unmounted")
            else:
                print(f"  Dirty flag:        clean")
            if bpb['io_error']:
                print(f"  *** I/O ERROR FLAG SET *** – disk surface errors detected")
            else:
                print(f"  I/O error flag:    clean")
            print()

            # FSInfo
            fsinfo = parse_fsinfo(raw_header, bpb)
            if fsinfo:
                print(f"{'─' * 60}")
                print("  FSInfo Sector")
                print(f"{'─' * 60}")
                print(f"  Valid:             {'YES' if fsinfo['valid'] else 'NO – CORRUPTED'}")
                if fsinfo['free_count'] != 0xFFFFFFFF:
                    free_mb = fsinfo['free_count'] * bpb['cluster_size'] / (1024 * 1024)
                    print(f"  Free Clusters:     {fsinfo['free_count']:,} ({free_mb:,.0f} MB)")
                print()

            # FAT comparison (if we read enough)
            if (bpb['num_fats'] >= 2 and
                len(raw_header) >= bpb['fat2_offset'] + bpb['fat_bytes']):
                print(f"{'─' * 60}")
                print("  FAT Table Comparison")
                print(f"{'─' * 60}")
                diffs, md5_1, md5_2 = compare_fats(raw_header, bpb)
                print(f"  FAT1 MD5:          {md5_1}")
                print(f"  FAT2 MD5:          {md5_2}")
                if diffs is not None:
                    if len(diffs) == 0:
                        print(f"  FAT1 vs FAT2:      IDENTICAL (good)")
                    else:
                        print(f"  *** FAT1 vs FAT2:  {len(diffs)} DIFFERENCES ***")
                        for idx, (entry, v1, v2) in enumerate(diffs[:20]):
                            print(f"    Entry {entry}: FAT1=0x{v1:08X} FAT2=0x{v2:08X}")
                            diff_bits = v1 ^ v2
                            if bin(diff_bits).count('1') == 1:
                                print(f"      ^ single bit flip (bit {diff_bits.bit_length() - 1})")
                        if len(diffs) > 20:
                            print(f"    ... and {len(diffs) - 20} more")
                print()
            elif bpb['num_fats'] >= 2:
                fat_needed = bpb['fat2_offset'] + bpb['fat_bytes']
                print(f"  NOTE: FAT comparison needs {fat_needed:,} bytes from raw device")
                print(f"        For full FAT analysis, image the card:")
                print(f"        sudo dd if={raw_dev} of=~/sd.img bs=4096 "
                      f"conv=noerror,sync status=progress")
                print()

    # ── Disk space (via statvfs) ──────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  Disk Space")
    print(f"{'─' * 60}")
    try:
        st = os.statvfs(vol_path)
        total = st.f_frsize * st.f_blocks
        free = st.f_frsize * st.f_bavail
        used = total - free
        print(f"  Total:   {total / (1024**2):,.0f} MB")
        print(f"  Used:    {used / (1024**2):,.0f} MB")
        print(f"  Free:    {free / (1024**2):,.0f} MB")
    except Exception as e:
        print(f"  Error: {e}")
    print()

    # ── Root directory listing ────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  Root Directory")
    print(f"{'─' * 60}")
    try:
        for name in sorted(os.listdir(vol_path)):
            if name.startswith('.'):
                continue
            full = os.path.join(vol_path, name)
            if os.path.isdir(full):
                print(f"  [DIR]  {name}")
            else:
                sz = os.path.getsize(full)
                print(f"  [FILE] {name:<30s}  {sz:>10,} bytes")
    except Exception as e:
        print(f"  Error listing root: {e}")
    print()

    # ── Log files ─────────────────────────────────────────────────────────
    logs_path = os.path.join(vol_path, 'logs')
    if not os.path.isdir(logs_path):
        print("  No /logs directory found")
        return

    print(f"{'─' * 60}")
    print("  /logs Directory")
    print(f"{'─' * 60}")

    all_logs = []
    try:
        for name in sorted(os.listdir(logs_path)):
            if name.startswith('.'):
                continue
            full = os.path.join(logs_path, name)
            if os.path.isfile(full):
                st = os.stat(full)
                all_logs.append({
                    'name': name,
                    'path': full,
                    'size': st.st_size,
                    'mtime': st.st_mtime,
                })
    except Exception as e:
        print(f"  Error listing logs: {e}")
        return

    adsb_files = [f for f in all_logs if f['name'].startswith('adsb_')]
    mesh_files = [f for f in all_logs if f['name'].startswith('mesh_')]
    other_files = [f for f in all_logs
                   if not f['name'].startswith('adsb_') and not f['name'].startswith('mesh_')]

    print(f"  ADS-B logs: {len(adsb_files)}")
    for f in adsb_files[-10:]:
        flag = ' !' if f['size'] == 0 else '  '
        print(f"  {flag} {f['name']:<45s} {f['size']:>8,} bytes")

    print(f"\n  Meshy logs: {len(mesh_files)}")
    for f in mesh_files[-10:]:
        flag = ' !' if f['size'] == 0 else '  '
        print(f"  {flag} {f['name']:<45s} {f['size']:>8,} bytes")

    if other_files:
        print(f"\n  Other files: {len(other_files)}")
        for f in other_files:
            print(f"     {f['name']:<45s} {f['size']:>8,} bytes")

    zero_files = [f for f in all_logs if f['size'] == 0]
    if zero_files:
        print(f"\n  *** {len(zero_files)} ZERO-SIZE LOG FILES ***")
        for f in zero_files:
            print(f"     {f['name']}")

    # ── Log file integrity ────────────────────────────────────────────────
    print(f"\n{'─' * 60}")
    print("  Log File Integrity")
    print(f"{'─' * 60}")

    files_to_check = adsb_files[-5:] + mesh_files[-5:]
    for entry in files_to_check:
        if entry['size'] == 0:
            print(f"\n  {entry['name']}: EMPTY (0 bytes)")
            continue

        try:
            with open(entry['path'], 'rb') as f:
                file_data = f.read()
        except Exception as e:
            print(f"\n  {entry['name']}: READ ERROR: {e}")
            continue

        info = analyze_log_file(file_data, entry['name'])

        status_icon = '✓' if info['status'] == 'OK' else '✗'
        print(f"\n  {status_icon} {info['filename']}  "
              f"({info['data_size']:,} bytes, {info.get('total_lines', 0)} lines)")

        if info.get('has_header'):
            print(f"    Header:        present")
        else:
            print(f"    Header:        MISSING")

        print(f"    Good lines:    {info.get('good_lines', '?')}")
        if info.get('bad_lines', 0) > 0:
            print(f"    *** Bad lines: {info['bad_lines']} ***")
        if info.get('truncated_end'):
            print(f"    *** TRUNCATED – no trailing newline ***")
        if info.get('embedded_nulls', 0) > 0:
            print(f"    *** {info['embedded_nulls']} NULL BYTES in data ***")
        if info.get('non_ascii_bytes', 0) > 0:
            print(f"    *** {info['non_ascii_bytes']} NON-ASCII bytes (bit flips?) ***")

        if args.dump_logs and info['status'] != 'EMPTY':
            text = file_data.rstrip(b'\x00').decode('utf-8', errors='replace')
            lines = [l for l in text.split('\n') if l.strip()]
            print(f"    Last 5 lines:")
            for line in lines[-5:]:
                print(f"      {line[:120]}")

    # ── Config file ───────────────────────────────────────────────────────
    config_path = os.path.join(vol_path, 'sd_config.txt')
    if os.path.isfile(config_path):
        print(f"\n{'─' * 60}")
        print("  sd_config.txt")
        print(f"{'─' * 60}")
        try:
            with open(config_path, 'r') as f:
                for line in f:
                    print(f"  {line.rstrip()}")
        except Exception as e:
            print(f"  Error reading config: {e}")

    # ── Summary ───────────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("  Summary")
    print(f"{'=' * 60}")

    issues = []
    if raw_header:
        bpb = parse_bpb(raw_header)
        if bpb:
            if bpb['dirty']:
                issues.append("FAT32 dirty flag set (not cleanly unmounted)")
            if bpb['io_error']:
                issues.append("FAT32 I/O error flag set (disk surface errors)")
    if zero_files:
        issues.append(f"{len(zero_files)} zero-size log files")

    if issues:
        print("  ISSUES FOUND:")
        for issue in issues:
            print(f"    • {issue}")
    else:
        print("  No obvious issues found")
    print()


# ── Raw image analysis ────────────────────────────────────────────────────────

def raw_analysis(path, args):
    """Analyze a raw SD card image or block device."""

    file_size = os.path.getsize(path)
    print(f"{'=' * 60}")
    print(f"  ADS-B Scope – SD Card Forensic Analysis")
    print(f"{'=' * 60}")
    print(f"  Image:  {path}")
    print(f"  Size:   {file_size:,} bytes ({file_size / (1024**3):.2f} GB)")
    print(f"  Mode:   raw image")
    print()

    # Read image
    read_size = args.max_read if args.max_read > 0 else file_size
    print(f"  Reading {read_size:,} bytes...")
    with open(path, 'rb') as f:
        data = f.read(read_size)
    print(f"  Read {len(data):,} bytes")
    print()

    # ── BPB Analysis ──────────────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  FAT32 Boot Parameter Block")
    print(f"{'─' * 60}")

    bpb = parse_bpb(data)
    if not bpb:
        print("  ERROR: Cannot parse BPB – is this a valid FAT32 image?")
        sys.exit(1)

    print(f"  OEM Name:          {bpb['oem_name']}")
    print(f"  Volume Label:      {bpb['volume_label']}")
    print(f"  FS Type:           {bpb['fs_type']}")
    print(f"  Bytes/Sector:      {bpb['bytes_per_sector']}")
    print(f"  Sectors/Cluster:   {bpb['sectors_per_cluster']}")
    print(f"  Cluster Size:      {bpb['cluster_size']:,} bytes")
    print(f"  Reserved Sectors:  {bpb['reserved_sectors']}")
    print(f"  Number of FATs:    {bpb['num_fats']}")
    print(f"  FAT Size:          {bpb['fat_size']} sectors ({bpb['fat_bytes']:,} bytes)")
    print(f"  Total Sectors:     {bpb['total_sectors']:,}")
    print(f"  Total Data Clusters: {bpb['total_data_clusters']:,}")
    print(f"  Root Cluster:      {bpb['root_cluster']}")
    print(f"  FSInfo Sector:     {bpb['fs_info_sector']}")
    print(f"  Data Start:        sector {bpb['data_start_sector']} "
          f"(offset 0x{bpb['data_start_offset']:X})")
    print()

    # ── Dirty Flag ────────────────────────────────────────────────────────
    dirty_byte = bpb['dirty_byte']
    print(f"  Dirty Byte (0x41): 0x{dirty_byte:02X} (binary: {dirty_byte:08b})")
    if bpb['dirty']:
        print(f"  *** DIRTY FLAG SET *** – filesystem was not cleanly unmounted")
    else:
        print(f"  Dirty flag:        clean")
    if bpb['io_error']:
        print(f"  *** I/O ERROR FLAG SET *** – disk surface errors detected")
    else:
        print(f"  I/O error flag:    clean")
    print()

    # ── FSInfo ────────────────────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  FSInfo Sector")
    print(f"{'─' * 60}")

    fsinfo = parse_fsinfo(data, bpb)
    if fsinfo:
        print(f"  Valid:             {'YES' if fsinfo['valid'] else 'NO – CORRUPTED'}")
        print(f"  Lead Signature:    0x{fsinfo['lead_sig']:08X} "
              f"({'OK' if fsinfo['lead_sig'] == 0x41615252 else 'BAD'})")
        print(f"  Struc Signature:   0x{fsinfo['struc_sig']:08X} "
              f"({'OK' if fsinfo['struc_sig'] == 0x61417272 else 'BAD'})")
        if fsinfo['free_count'] != 0xFFFFFFFF:
            free_mb = fsinfo['free_count'] * bpb['cluster_size'] / (1024 * 1024)
            print(f"  Free Clusters:     {fsinfo['free_count']:,} ({free_mb:,.0f} MB)")
        else:
            print(f"  Free Clusters:     unknown (0xFFFFFFFF)")
        print(f"  Next Free:         {fsinfo['next_free']}")
    else:
        print("  WARNING: Cannot read FSInfo sector")
    print()

    # ── FAT Comparison ────────────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  FAT Table Comparison")
    print(f"{'─' * 60}")

    diffs = None
    if bpb['num_fats'] >= 2 and len(data) >= bpb['fat2_offset'] + bpb['fat_bytes']:
        diffs, md5_1, md5_2 = compare_fats(data, bpb)
        print(f"  FAT1 MD5:          {md5_1}")
        print(f"  FAT2 MD5:          {md5_2}")
        if diffs is not None:
            if len(diffs) == 0:
                print(f"  FAT1 vs FAT2:      IDENTICAL (good)")
            else:
                print(f"  *** FAT1 vs FAT2:  {len(diffs)} DIFFERENCES ***")
                for idx, (entry, v1, v2) in enumerate(diffs[:20]):
                    print(f"    Entry {entry}: FAT1=0x{v1:08X} FAT2=0x{v2:08X}")
                    # Check for bit-flip
                    diff = v1 ^ v2
                    if bin(diff).count('1') == 1:
                        print(f"      ^ single bit flip (bit {diff.bit_length() - 1})")
                if len(diffs) > 20:
                    print(f"    ... and {len(diffs) - 20} more differences")
    else:
        print("  WARNING: Image too small to compare both FATs")
    print()

    # ── FAT Analysis ──────────────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  FAT1 Cluster Analysis")
    print(f"{'─' * 60}")

    fat1 = read_fat(data, bpb, 0)
    fat_stats = analyze_fat(fat1, bpb)
    print(f"  Total clusters:    {fat_stats['total_entries']:,}")
    print(f"  Free:              {fat_stats['free']:,} "
          f"({fat_stats['free'] * bpb['cluster_size'] / (1024**2):,.0f} MB)")
    print(f"  Used:              {fat_stats['used']:,} "
          f"({fat_stats['used'] * bpb['cluster_size'] / (1024**2):,.0f} MB)")
    print(f"  Bad:               {fat_stats['bad']:,}")
    print(f"  Chain endings:     {fat_stats['eoc_count']:,} (≈ files + dirs)")
    print(f"  Chain heads:       {fat_stats['chain_heads']:,}")
    if fat_stats['bad'] > 0:
        print(f"  *** BAD CLUSTERS FOUND – card may have physical damage ***")
    print()

    # ── Root Directory ────────────────────────────────────────────────────
    print(f"{'─' * 60}")
    print("  Root Directory")
    print(f"{'─' * 60}")

    root_data = read_cluster_chain(data, fat1, bpb['root_cluster'], bpb)
    root_entries = parse_directory(root_data)

    for entry in root_entries:
        if entry['is_dir']:
            print(f"  [DIR]  {entry['name']:<30s}  cluster={entry['cluster']}")
        else:
            print(f"  [FILE] {entry['name']:<30s}  {entry['size']:>10,} bytes  "
                  f"mod={entry['modified']}")
    print()

    # ── /logs directory ───────────────────────────────────────────────────
    logs_dir = None
    for entry in root_entries:
        if entry['is_dir'] and entry['name'].lower() == 'logs':
            logs_dir = entry
            break

    if logs_dir:
        print(f"{'─' * 60}")
        print("  /logs Directory")
        print(f"{'─' * 60}")

        logs_data = read_cluster_chain(data, fat1, logs_dir['cluster'], bpb)
        log_entries = parse_directory(logs_data)

        # Sort by name
        log_entries.sort(key=lambda e: e['name'])

        adsb_files = []
        mesh_files = []
        other_files = []

        for entry in log_entries:
            if entry['name'].startswith('.'):
                continue
            if entry['is_dir']:
                continue
            if entry['name'].startswith('adsb_'):
                adsb_files.append(entry)
            elif entry['name'].startswith('mesh_'):
                mesh_files.append(entry)
            else:
                other_files.append(entry)

        print(f"  ADS-B logs: {len(adsb_files)}")
        for f in adsb_files[-10:]:  # last 10
            flag = ' !' if f['size'] == 0 else '  '
            print(f"  {flag} {f['name']:<45s} {f['size']:>8,} bytes  "
                  f"mod={f['modified']}")

        print(f"\n  Meshy logs: {len(mesh_files)}")
        for f in mesh_files[-10:]:
            flag = ' !' if f['size'] == 0 else '  '
            print(f"  {flag} {f['name']:<45s} {f['size']:>8,} bytes  "
                  f"mod={f['modified']}")

        if other_files:
            print(f"\n  Other files: {len(other_files)}")
            for f in other_files:
                print(f"     {f['name']:<45s} {f['size']:>8,} bytes")

        # ── Zero-size file analysis ───────────────────────────────────────
        zero_files = [f for f in log_entries if f['size'] == 0 and not f['is_dir']
                      and not f['name'].startswith('.')]
        if zero_files:
            print(f"\n  *** {len(zero_files)} ZERO-SIZE LOG FILES ***")
            for f in zero_files:
                print(f"     {f['name']}  created={f['created']}  cluster={f['cluster']}")

        # ── Log file content analysis ─────────────────────────────────────
        print(f"\n{'─' * 60}")
        print("  Log File Integrity")
        print(f"{'─' * 60}")

        # Analyze last few ADS-B and Meshy files
        files_to_check = adsb_files[-5:] + mesh_files[-5:]
        for entry in files_to_check:
            if entry['size'] == 0:
                print(f"\n  {entry['name']}: EMPTY (0 bytes)")
                continue

            file_data = read_cluster_chain(data, fat1, entry['cluster'], bpb)
            # Trim to actual file size
            file_data = file_data[:entry['size']]

            info = analyze_log_file(file_data, entry['name'])

            status_icon = '✓' if info['status'] == 'OK' else '✗'
            print(f"\n  {status_icon} {info['filename']}  "
                  f"({info['data_size']:,} bytes, {info.get('total_lines', 0)} lines)")

            if info.get('has_header'):
                print(f"    Header:        present")
            else:
                print(f"    Header:        MISSING")

            print(f"    Good lines:    {info.get('good_lines', '?')}")
            if info.get('bad_lines', 0) > 0:
                print(f"    *** Bad lines: {info['bad_lines']} ***")
            if info.get('truncated_end'):
                print(f"    *** TRUNCATED – no trailing newline ***")
            if info.get('embedded_nulls', 0) > 0:
                print(f"    *** {info['embedded_nulls']} NULL BYTES in data ***")
            if info.get('non_ascii_bytes', 0) > 0:
                print(f"    *** {info['non_ascii_bytes']} NON-ASCII bytes (bit flips?) ***")

            if args.dump_logs and info['status'] != 'EMPTY':
                text = file_data[:entry['size']].rstrip(b'\x00').decode('utf-8', errors='replace')
                lines = [l for l in text.split('\n') if l.strip()]
                print(f"    Last 5 lines:")
                for line in lines[-5:]:
                    print(f"      {line[:120]}")

    # ── SD Config file ────────────────────────────────────────────────────
    config_dir = None
    for entry in root_entries:
        if entry['name'].lower() in ('config', 'data'):
            if entry['is_dir']:
                config_dir = entry
                break

    # Look for sd_config in root
    for entry in root_entries:
        if entry['name'] == 'sd_config.txt' or entry['name'] == '.sd_config':
            print(f"\n{'─' * 60}")
            print(f"  Config File: {entry['name']}")
            print(f"{'─' * 60}")
            file_data = read_cluster_chain(data, fat1, entry['cluster'], bpb)
            text = file_data[:entry['size']].decode('utf-8', errors='replace')
            for line in text.strip().split('\n'):
                print(f"  {line}")

    # ── Summary ───────────────────────────────────────────────────────────
    print(f"\n{'=' * 60}")
    print("  Summary")
    print(f"{'=' * 60}")

    issues = []
    if bpb['dirty']:
        issues.append("FAT32 dirty flag set (not cleanly unmounted)")
    if bpb['io_error']:
        issues.append("FAT32 I/O error flag set (disk surface errors)")
    if diffs and len(diffs) > 0:
        issues.append(f"FAT1 vs FAT2 mismatch ({len(diffs)} entries)")
    if fat_stats['bad'] > 0:
        issues.append(f"{fat_stats['bad']} bad clusters")
    if logs_dir:
        if zero_files:
            issues.append(f"{len(zero_files)} zero-size log files")

    if issues:
        print("  ISSUES FOUND:")
        for issue in issues:
            print(f"    • {issue}")
    else:
        print("  No obvious issues found")

    print()
    print(f"  Tip: run with --dump-logs to see last lines of each log file")
    print()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='ADS-B Scope SD Card Forensics',
        epilog="""Examples:
  %(prog)s /Volumes/SD32G              Live analysis of mounted card
  %(prog)s /Volumes/SD32G --dump-logs  Live + show last lines of logs
  %(prog)s ~/sd_failed.img             Raw image analysis
  %(prog)s --find                      Auto-detect ADS-B Scope SD cards
  sudo %(prog)s /Volumes/SD32G         Live + BPB/FAT via raw device""",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('path', nargs='?', default=None,
                        help='Path to SD image (.img), block device, '
                             'or mounted volume (/Volumes/...)')
    parser.add_argument('--dump-logs', action='store_true',
                        help='Dump last 5 lines of each log file')
    parser.add_argument('--max-read', type=int, default=0,
                        help='Max bytes to read from raw image (0 = all)')
    parser.add_argument('--find', action='store_true',
                        help='Auto-detect ADS-B Scope SD cards in /Volumes')
    args = parser.parse_args()

    # Auto-detect mode
    if args.find or args.path is None:
        vols = find_sd_volumes()
        if not vols:
            if args.find:
                print("No ADS-B Scope SD cards found in /Volumes/")
                print("Insert a card or specify a path manually.")
            else:
                parser.print_help()
            sys.exit(1 if args.find else 0)

        if len(vols) == 1:
            vol_path = vols[0][0]
            print(f"Found ADS-B Scope SD card: {vol_path}")
            live_analysis(vol_path, args)
            return

        print("Found multiple ADS-B Scope SD cards:")
        for i, (path, name, has_logs, has_config, has_music) in enumerate(vols):
            markers = []
            if has_logs: markers.append('logs')
            if has_config: markers.append('config')
            if has_music: markers.append('music')
            print(f"  [{i + 1}] {path}  ({', '.join(markers)})")
        print(f"\nSpecify one: python3 {sys.argv[0]} /Volumes/<name>")
        return

    path = args.path
    if not os.path.exists(path):
        print(f"Error: {path} not found")
        sys.exit(1)

    # Directory = mounted volume = live mode
    if os.path.isdir(path):
        live_analysis(path, args)
    else:
        raw_analysis(path, args)


if __name__ == '__main__':
    main()
