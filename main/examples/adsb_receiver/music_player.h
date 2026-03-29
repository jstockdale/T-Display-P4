/*
 * music_player.h — SD card music player for T-Display-P4
 *
 * Supports MP3 (via minimp3) and WAV (44100Hz 16-bit stereo).
 * Scans /sdcard/music/ for tracks, parses ID3v2 tags for metadata.
 *
 * Usage:
 *   music_player_init();           // call once at boot
 *   music_player_scan();           // scan SD card for tracks
 *   music_player_play();           // start/resume current track
 *   music_player_pause();          // pause playback
 *   music_player_next();           // advance to next track
 *   music_player_prev();           // go to previous track
 *   music_player_stop();           // stop and reset
 */
#ifndef MUSIC_PLAYER_H
#define MUSIC_PLAYER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MUSIC_DIR "/sdcard/music"
#define MUSIC_MAX_TRACKS   64
#define MUSIC_TITLE_LEN    64
#define MUSIC_ARTIST_LEN   64
#define MUSIC_PATH_LEN    272

typedef enum {
    MUSIC_FORMAT_UNKNOWN = 0,
    MUSIC_FORMAT_MP3,
    MUSIC_FORMAT_WAV,
} music_format_t;

typedef enum {
    MUSIC_STATE_STOPPED = 0,
    MUSIC_STATE_PLAYING,
    MUSIC_STATE_PAUSED,
} music_state_t;

typedef struct {
    char            path[MUSIC_PATH_LEN];
    char            title[MUSIC_TITLE_LEN];
    char            artist[MUSIC_ARTIST_LEN];
    music_format_t  format;
    double          duration_s;      // 0 until playback starts (MP3 may not know upfront)
} music_track_t;

typedef struct {
    music_state_t   state;
    int             track_index;     // -1 if no tracks
    int             track_count;
    double          position_s;      // current playback position
    double          duration_s;      // total duration of current track
    const char     *title;           // points into track array (do not free)
    const char     *artist;
} music_player_info_t;

// Initialize the music player (allocates state, call once)
void music_player_init(void);

// Scan /sdcard/music/ for .mp3 and .wav files.
// Populates playlist. Returns number of tracks found.
int music_player_scan(void);

// Get current player state + track info
music_player_info_t music_player_get_info(void);

// Get track info by index (NULL if out of range)
const music_track_t *music_player_get_track(int index);

// Get track count
int music_player_track_count(void);

// Set current track index (does not start playback)
void music_player_set_track(int index);

// Start or resume playback of current track.
// This is a BLOCKING call — runs the decode+write loop until
// the track ends, stop is requested, or next/prev is triggered.
// Call from the speaker task.
void music_player_play_blocking(void);

// Request stop (non-blocking, signals the blocking play loop to exit)
void music_player_stop(void);

// Request pause (non-blocking)
void music_player_pause(void);

// Request resume (non-blocking, only if paused)
void music_player_resume(void);

// Advance to next track (wraps). If playing, restarts with new track.
void music_player_next(void);

// Go to previous track (wraps). If playing, restarts with new track.
void music_player_prev(void);

// Seek to position in seconds (approximate for MP3)
void music_player_seek(double position_s);

// Reconfigure I2S + codec output rate (called automatically during playback).
// Implemented in main.cpp as a bridge to ES8311 + I2S bus.
bool music_set_output_rate(uint32_t rate_hz);

// Get embedded album art (JPEG/PNG) for current track.
// Returns pointer to raw image data in PSRAM, or NULL if none.
// Caller must NOT free the returned pointer — it's a persistent buffer.
const uint8_t *music_player_get_album_art(size_t *out_size);

// Extract album art from a specific track (by index).
// Loads APIC data into persistent PSRAM buffer.
// Returns true if art was found.
bool music_player_load_album_art(int track_index);

// Check if a track change was requested (next/prev during playback).
// The play loop checks this to know when to restart.
bool music_player_track_changed(void);

// Clear track-changed flag
void music_player_ack_track_change(void);

// Close all SD file handles (waits for in-flight reads to finish).
// Call before unmounting SD card to avoid SPI bus spinlock crash.
void music_player_sd_close(void);

#ifdef __cplusplus
}
#endif

#endif // MUSIC_PLAYER_H
