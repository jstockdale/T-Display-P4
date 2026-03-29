# Music — ADS-B Scope

The T-Display-P4 plays MP3 files from this folder through the onboard ES8311 speaker.

## Quick start

1. Copy `.mp3` files into `/sdcard/music/`
2. Run `prepare_music.py` on your computer (optional but recommended)
3. Reboot the device — tracks appear in the music player

## Supported formats

**MP3** — Any bitrate (CBR or VBR), mono or stereo, 8–48 kHz sample rate. Stereo is
automatically downmixed to mono for the ES8311 codec. Decoding is handled by minimp3
(public domain, runs entirely on the ESP32-P4).

**WAV** — 16-bit PCM, 44100 Hz, mono or stereo. Large file sizes make WAV impractical
on SD cards — MP3 is recommended.

Up to **64 tracks** are loaded per scan. Files are sorted alphabetically by filename.

## Album art

The player displays embedded cover art on the AMOLED screen (544×544 display area).
Art is read from the **APIC** frame in the MP3's ID3v2 tags — this is the standard
way iTunes, foobar2000, MusicBrainz Picard, and most taggers embed artwork.

For best results:

- JPEG format, square aspect ratio
- 1024×1024 or smaller (the device downscales to 544×544)
- Avoid oversized art (4000×4000 originals waste SD read time and PSRAM)

If your files don't have embedded art, any ID3 tag editor can add it —
MusicBrainz Picard, Mp3tag (Windows), Kid3, or foobar2000 all work.

## prepare_music.py

This script optimizes your MP3 files for the device. It's optional but makes the
experience noticeably better.

**What it does:**

- Writes an exact **TLEN** duration tag so the progress bar works correctly from the
  start of playback (without it, duration is estimated from bitrate — inaccurate for VBR)
- **Normalizes Unicode** in title/artist tags to ASCII — converts smart quotes, em dashes,
  accented characters, etc. so they render correctly on the ESP32's text display
- **Resizes oversized album art** to max 1024×1024 JPEG — saves SD read time without
  visible quality loss on the 544px display

**Requirements:**

```
pip install mutagen Pillow
```

**Usage:**

```
# Single file
python3 prepare_music.py song.mp3

# All MP3s in a directory
python3 prepare_music.py /path/to/music/*.mp3

# Current directory
python3 prepare_music.py .
```

The script modifies files in place — it does not create copies. Your audio data is
never re-encoded; only the ID3 metadata and embedded images are touched.

## Metadata

Track title and artist are read from ID3v2 **TIT2** and **TPE1** tags. If no tags are
found, the filename is used as the title (minus the extension, with underscores and
hyphens converted to spaces).

## File naming

- Files starting with `.` are ignored (macOS `.DS_Store`, `._` resource forks)
- Files containing `~` are ignored (Windows 8.3 short name aliases like `THEYD~3.MP3`)
- Subdirectories are not scanned — all tracks must be directly in `/sdcard/music/`

## Playback controls

On the device, use the music player screen to play/pause, skip tracks, and seek.
Volume is controlled through the device settings.
