// screen_detect.h — Runtime display variant detection for T-Display-P4
// Supports HI8561 (540×1168 LCD) and RM69A10 (568×1232 AMOLED)
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "t_display_p4_config.h"

typedef enum {
    SCREEN_TYPE_UNKNOWN = 0,
    SCREEN_TYPE_HI8561,     // 540×1168 LCD (integrated touch at I2C 0x68)
    SCREEN_TYPE_RM69A10,    // 568×1232 AMOLED (GT9895 touch at I2C 0x5D)
} screen_type_t;

#ifdef __cplusplus
extern "C" {
#endif

extern screen_type_t g_screen_type;
extern uint32_t g_screen_width;
extern uint32_t g_screen_height;

// Max dimensions across all variants — use for buffer allocation
#define SCREEN_WIDTH_MAX   RM69A10_SCREEN_WIDTH    // 568
#define SCREEN_HEIGHT_MAX  RM69A10_SCREEN_HEIGHT   // 1232

static inline bool screen_is_hi8561(void)  { return g_screen_type == SCREEN_TYPE_HI8561; }
static inline bool screen_is_rm69a10(void) { return g_screen_type == SCREEN_TYPE_RM69A10; }

static inline const char *screen_type_name(void) {
    switch (g_screen_type) {
        case SCREEN_TYPE_HI8561:  return "HI8561 LCD (540x1168)";
        case SCREEN_TYPE_RM69A10: return "RM69A10 AMOLED (568x1232)";
        default:                  return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif
