// emoji_draw.h — Emoji drawing utilities for LVGL canvas + UTF-8 text
// Include emoji_sprites.h before this file.
#pragma once

#include <stdint.h>
#include <string.h>
#include "lvgl.h"
#include "esp_heap_caps.h"

// --- UTF-8 decoder ---
// Decodes one codepoint from a UTF-8 string, advances the pointer.
// Returns 0 on end-of-string or invalid sequence.
static inline uint32_t utf8_decode(const char **p) {
    const uint8_t *s = (const uint8_t *)*p;
    if (!*s) return 0;

    uint32_t cp;
    int extra;

    if (*s < 0x80)        { cp = *s;         extra = 0; }
    else if (*s < 0xC0)   { *p = (const char *)s + 1; return 0xFFFD; } // invalid continuation
    else if (*s < 0xE0)   { cp = *s & 0x1F;  extra = 1; }
    else if (*s < 0xF0)   { cp = *s & 0x0F;  extra = 2; }
    else if (*s < 0xF8)   { cp = *s & 0x07;  extra = 3; }
    else                  { *p = (const char *)s + 1; return 0xFFFD; }

    s++;
    for (int i = 0; i < extra; i++) {
        if ((*s & 0xC0) != 0x80) { *p = (const char *)s; return 0xFFFD; }
        cp = (cp << 6) | (*s & 0x3F);
        s++;
    }

    // Skip variant selector (U+FE0F) if present after emoji
    if (s[0] == 0xEF && s[1] == 0xB8 && s[2] == 0x8F) s += 3;

    *p = (const char *)s;
    return cp;
}

// --- Canvas emoji blitter ---
// Draws an emoji sprite directly onto an RGB565 buffer at (x, y).
// Supports both binary blob format and header-only format.
// Binary blob: emoji_sprites.h defines emoji_rgb565() / emoji_alpha()
// Header-only: emoji_sprites.h has .rgb565 / .alpha pointers directly
static inline int draw_emoji_on_buffer(uint16_t *buf, int32_t buf_w, int32_t buf_h,
                                        uint32_t codepoint, int32_t x, int32_t y) {
    const emoji_sprite_t *e = emoji_find(codepoint);
    if (!e) return 0;

    const uint16_t *src_rgb;
    const uint8_t  *src_a;
#if defined(EMOJI_BINARY_BLOB)
    src_rgb = emoji_rgb565(e);
    src_a   = emoji_alpha(e);
#else
    src_rgb = e->rgb565;
    src_a   = e->alpha;
#endif

    for (int row = 0; row < e->h; row++) {
        for (int col = 0; col < e->w; col++) {
            int32_t px = x + col;
            int32_t py = y + row;
            if (px < 0 || px >= buf_w || py < 0 || py >= buf_h) continue;

            int idx = row * e->w + col;
            uint8_t alpha = src_a[idx];
            if (alpha < 32) continue;

            uint16_t src = src_rgb[idx];

            if (alpha >= 224) {
                buf[py * buf_w + px] = src;
            } else {
                uint16_t bg = buf[py * buf_w + px];
                uint8_t sr = ((src >> 11) & 0x1F) << 3;
                uint8_t sg = ((src >> 5) & 0x3F) << 2;
                uint8_t sb = (src & 0x1F) << 3;
                uint8_t br = ((bg >> 11) & 0x1F) << 3;
                uint8_t bgr = ((bg >> 5) & 0x3F) << 2;
                uint8_t bb = (bg & 0x1F) << 3;
                uint8_t r = (sr * alpha + br * (255 - alpha)) / 255;
                uint8_t g = (sg * alpha + bgr * (255 - alpha)) / 255;
                uint8_t b = (sb * alpha + bb * (255 - alpha)) / 255;
                buf[py * buf_w + px] = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            }
        }
    }
    return e->w;
}

// --- Check if a codepoint is a known emoji ---
static inline bool is_known_emoji(uint32_t cp) {
    return emoji_find(cp) != NULL;
}

// --- Draw text with inline emoji onto LVGL canvas ---
// Draws text at (x, y) using the given font for regular characters,
// substituting emoji sprites where found. Returns total width drawn.
// buf/buf_w/buf_h: direct canvas buffer access for emoji blitting.
//
// IMPORTANT: Uses a pool of persistent text buffers because LVGL 9 canvas
// draw is deferred — dsc.text must survive until lv_canvas_finish_layer().
// Call draw_text_emoji_reset_pool() before starting a new frame.
// Pool lives in PSRAM to avoid consuming scarce internal RAM.
#define EMOJI_TEXT_POOL_SLOTS 200
#define EMOJI_TEXT_POOL_SLOT_SIZE 128
static char (*_emoji_text_pool)[EMOJI_TEXT_POOL_SLOT_SIZE] = nullptr;
static int _emoji_text_pool_idx = 0;

static inline void draw_text_emoji_reset_pool(void) {
    _emoji_text_pool_idx = 0;
    if (!_emoji_text_pool) {
        _emoji_text_pool = (char (*)[EMOJI_TEXT_POOL_SLOT_SIZE])heap_caps_calloc(
            EMOJI_TEXT_POOL_SLOTS, EMOJI_TEXT_POOL_SLOT_SIZE, MALLOC_CAP_SPIRAM);
    }
}

static inline char *_emoji_pool_alloc(const char *src, int len) {
    if (!_emoji_text_pool || _emoji_text_pool_idx >= EMOJI_TEXT_POOL_SLOTS) return (char *)"?";
    if (len > EMOJI_TEXT_POOL_SLOT_SIZE - 1) len = EMOJI_TEXT_POOL_SLOT_SIZE - 1;
    char *slot = _emoji_text_pool[_emoji_text_pool_idx++];
    memcpy(slot, src, len);
    slot[len] = '\0';
    return slot;
}

static inline int draw_text_with_emoji(uint16_t *buf, int32_t buf_w, int32_t buf_h,
                                        lv_layer_t *layer,
                                        const char *text, int32_t x, int32_t y,
                                        const lv_font_t *font, lv_color_t color) {
    int32_t cursor_x = x;
    int32_t max_x = buf_w - 4;  // right margin
    const char *p = text;
    int font_h = lv_font_get_line_height(font);

    const char *run_start = p;

    while (*p) {
        const char *before = p;
        uint32_t cp = utf8_decode(&p);
        if (cp == 0) break;

        const emoji_sprite_t *emoji = emoji_find(cp);
        if (emoji) {
            if (before > run_start) {
                int len = before - run_start;
                char *persistent = _emoji_pool_alloc(run_start, len);

                lv_draw_label_dsc_t dsc;
                lv_draw_label_dsc_init(&dsc);
                dsc.color = color;
                dsc.font = font;
                dsc.opa = LV_OPA_COVER;
                dsc.text = persistent;
                lv_area_t area = { cursor_x, y, max_x, y + font_h };
                lv_draw_label(layer, &dsc, &area);

                lv_point_t txt_size;
                lv_text_get_size(&txt_size, persistent, font, 0, 0, max_x - cursor_x, LV_TEXT_FLAG_NONE);
                cursor_x += txt_size.x;
            }

            int32_t ey = y + (font_h - emoji->h) / 2;
            draw_emoji_on_buffer(buf, buf_w, buf_h, cp, cursor_x, ey);
            cursor_x += emoji->w + 2;

            run_start = p;
        }
    }

    if (p > run_start) {
        int len = p - run_start;
        char *persistent = _emoji_pool_alloc(run_start, len);

        lv_draw_label_dsc_t dsc;
        lv_draw_label_dsc_init(&dsc);
        dsc.color = color;
        dsc.font = font;
        dsc.opa = LV_OPA_COVER;
        dsc.text = persistent;
        lv_area_t area = { cursor_x, y, max_x, y + font_h };
        lv_draw_label(layer, &dsc, &area);

        lv_point_t txt_size;
        lv_text_get_size(&txt_size, persistent, font, 0, 0, max_x - cursor_x, LV_TEXT_FLAG_NONE);
        cursor_x += txt_size.x;
    }

    return cursor_x - x;
}
