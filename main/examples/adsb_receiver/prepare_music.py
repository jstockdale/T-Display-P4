#!/usr/bin/env python3
"""
prepare_music.py — Prepare MP3 files for ADS-B Scope T-Display-P4.

For each MP3 file:
  1. Writes the TLEN ID3 tag (exact duration in ms) so the firmware doesn't
     have to estimate duration from bitrate.  Uses mutagen's frame-accurate
     duration which counts actual MPEG frames.
  2. Normalizes Unicode text in title/artist tags to ASCII — converts smart
     quotes, em dashes, accented characters etc. so they display correctly
     on the ESP32's simple text renderer.
  3. Resizes embedded album art to max 1024×1024 JPEG if oversized.
     The ESP32-P4 downscales to 544×544 on-device, but huge source images
     waste SD read time and PSRAM.

Usage:
    python3 prepare_music.py <file.mp3> [file2.mp3 ...]
    python3 prepare_music.py /path/to/music/*.mp3
    python3 prepare_music.py .   # process all .mp3 in current directory

Requires: pip install mutagen Pillow
"""

import sys
import io
import unicodedata
from pathlib import Path

from mutagen.mp3 import MP3
from mutagen.id3 import ID3, APIC, TLEN, TIT2, TPE1, ID3NoHeaderError
from PIL import Image

MAX_ART_SIZE = 1024
JPEG_QUALITY = 82

# Unicode → ASCII substitutions for characters the ESP32 can't render.
# Covers smart quotes, dashes, ellipsis, and other common typographic chars.
UNICODE_MAP = str.maketrans({
    '\u2018': "'",   # '  left single quote
    '\u2019': "'",   # '  right single quote / apostrophe
    '\u201A': "'",   # ‚  single low-9 quote
    '\u201B': "'",   # ‛  single high-reversed-9 quote
    '\u201C': '"',   # "  left double quote
    '\u201D': '"',   # "  right double quote
    '\u201E': '"',   # „  double low-9 quote
    '\u201F': '"',   # ‟  double high-reversed-9 quote
    '\u2032': "'",   # ′  prime
    '\u2033': '"',   # ″  double prime
    '\u2010': '-',   # ‐  hyphen
    '\u2011': '-',   # ‑  non-breaking hyphen
    '\u2012': '-',   # ‒  figure dash
    '\u2013': '-',   # –  en dash
    '\u2014': '-',   # —  em dash
    '\u2015': '-',   # ―  horizontal bar
    '\u2026': '...', # …  ellipsis
    '\u00A0': ' ',   #    non-breaking space
    '\u00AB': '"',   # «  left guillemet
    '\u00BB': '"',   # »  right guillemet
    '\u2039': "'",   # ‹  single left guillemet
    '\u203A': "'",   # ›  single right guillemet
    '\u00B7': '.',   # ·  middle dot
    '\u2022': '*',   # •  bullet
    '\u00D7': 'x',   # ×  multiplication sign
    '\u2044': '/',   # ⁄  fraction slash
    '\u00E9': 'e',   # é  (common in music)
    '\u00E8': 'e',   # è
    '\u00F1': 'n',   # ñ
    '\u00FC': 'u',   # ü
    '\u00E4': 'a',   # ä
    '\u00F6': 'o',   # ö
})


def normalize_text(s: str) -> str:
    """Normalize Unicode text to ASCII-safe characters for ESP32 display."""
    # Apply explicit substitution map first
    s = s.translate(UNICODE_MAP)
    # Then use Unicode NFKD decomposition to strip remaining accents/diacritics
    # that weren't in our explicit map (e.g. ó → o, ç → c)
    decomposed = unicodedata.normalize('NFKD', s)
    ascii_safe = ''.join(c for c in decomposed if unicodedata.category(c) != 'Mn')
    # Drop any remaining non-ASCII that slipped through
    return ascii_safe.encode('ascii', errors='replace').decode('ascii')


def process_file(path: Path) -> None:
    print(f"\n  {path.name}")

    # ── Load MP3 and get accurate duration ──────────────────────────────
    try:
        audio = MP3(str(path))
    except Exception as e:
        print(f"    SKIP — cannot read MP3 ({e})")
        return

    duration_ms = int(audio.info.length * 1000)
    duration_s = audio.info.length

    # Ensure ID3 tags exist
    try:
        tags = ID3(str(path))
    except ID3NoHeaderError:
        print(f"    Adding ID3 header")
        audio.add_tags()
        audio.save()
        tags = ID3(str(path))

    modified = False

    # ── TLEN: write exact duration ──────────────────────────────────────
    existing_tlen = tags.get("TLEN")
    if existing_tlen:
        old_ms = int(str(existing_tlen))
        # Only update if off by more than 500ms
        if abs(old_ms - duration_ms) > 500:
            print(f"    TLEN  {old_ms}ms → {duration_ms}ms ({duration_s:.1f}s)")
            tags.delall("TLEN")
            tags.add(TLEN(encoding=0, text=[str(duration_ms)]))
            modified = True
        else:
            print(f"    TLEN  OK ({duration_ms}ms = {duration_s:.1f}s)")
    else:
        print(f"    TLEN  added: {duration_ms}ms ({duration_s:.1f}s)")
        tags.add(TLEN(encoding=0, text=[str(duration_ms)]))
        modified = True

    # ── Text tags: normalize Unicode to ASCII ─────────────────────────
    for tag_id, tag_class, label in [("TIT2", TIT2, "title"), ("TPE1", TPE1, "artist")]:
        frame = tags.get(tag_id)
        if frame:
            original = str(frame)
            normalized = normalize_text(original)
            if normalized != original:
                tags.delall(tag_id)
                tags.add(tag_class(encoding=3, text=[normalized]))
                modified = True
                print(f"    {tag_id}  \"{original}\" → \"{normalized}\"")
            else:
                print(f"    {tag_id}  OK \"{original}\"")
        else:
            print(f"    {tag_id}  (not set)")

    # ── Album art: resize if oversized ──────────────────────────────────
    apic_keys = [k for k in tags if k.startswith("APIC")]
    if not apic_keys:
        print(f"    ART   no embedded art")
    else:
        for key in apic_keys:
            frame = tags[key]
            try:
                img = Image.open(io.BytesIO(frame.data))
            except Exception as e:
                print(f"    ART   cannot decode ({e})")
                continue

            w, h = img.size
            art_kb = len(frame.data) / 1024

            if w <= MAX_ART_SIZE and h <= MAX_ART_SIZE:
                print(f"    ART   OK {w}×{h} ({art_kb:.0f} KB)")
                continue

            # Resize proportionally
            scale = MAX_ART_SIZE / max(w, h)
            new_w = int(w * scale)
            new_h = int(h * scale)

            img_resized = img.resize((new_w, new_h), Image.LANCZOS)
            buf = io.BytesIO()
            img_resized.convert("RGB").save(buf, format="JPEG",
                                            quality=JPEG_QUALITY, optimize=True)
            new_data = buf.getvalue()
            new_kb = len(new_data) / 1024

            frame.data = new_data
            frame.mime = "image/jpeg"
            modified = True
            print(f"    ART   {w}×{h} ({art_kb:.0f} KB) → {new_w}×{new_h} ({new_kb:.0f} KB)")

    # ── Save ────────────────────────────────────────────────────────────
    if modified:
        tags.save(str(path), v2_version=3)
        print(f"    SAVED")
    else:
        print(f"    (no changes)")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <file.mp3|directory> [...]")
        print(f"  Writes TLEN (exact duration) and resizes album art for ESP32 playback.")
        sys.exit(1)

    files = []
    for arg in sys.argv[1:]:
        p = Path(arg)
        if p.is_dir():
            files.extend(sorted(p.glob("*.mp3")))
            files.extend(sorted(p.glob("*.MP3")))
        elif p.is_file() and p.suffix.lower() == ".mp3":
            files.append(p)
        elif "*" in arg:
            files.extend(sorted(Path(".").glob(arg)))

    if not files:
        print("No MP3 files found.")
        sys.exit(1)

    print(f"Processing {len(files)} file(s)...")
    for f in files:
        process_file(f)

    print(f"\nDone — {len(files)} file(s) processed.")


if __name__ == "__main__":
    main()
