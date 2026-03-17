/*
 * serial_console.h – Interactive serial console for ADS-B receiver
 *
 * Two modes:
 *   LOG  – streams ADS-B/GNSS output (default, current behavior)
 *   CMD  – interactive prompt with file access and control commands
 *
 * Ctrl+C escapes from LOG to CMD mode.
 * "log" command returns to LOG mode.
 */
#ifndef SERIAL_CONSOLE_H
#define SERIAL_CONSOLE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Call once from app_main after all peripherals are initialized
void serial_console_init(void);

// Check if log output is currently enabled (LOG mode)
// ADS-B and GNSS printf calls should check this before printing
bool serial_console_log_enabled(void);

// Call from any task to print output that respects the console mode.
// In LOG mode, prints immediately. In CMD mode, suppresses (or buffers).
// Returns true if the line was printed.
bool serial_console_print(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif // SERIAL_CONSOLE_H
