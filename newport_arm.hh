#ifndef __NEWPORT_ARM_HH__
#define __NEWPORT_ARM_HH__
#include <cstdint>

class Driver;

/* Newport behind henry's ARM-served GIO window (rtl/gio_window.sv,
 * docs/newport-on-henry.md): a service thread pinned to `cpu` drains the capture ring
 * in the staging mapping into sgi_newport and answers the held loads; with a non-empty
 * `display` (e.g. ":2") a second thread shows the frame in an X11 window. */
void newport_arm_start(Driver *d, uint8_t *stg, const char *display, int cpu);
/* stop both threads (before the staging mapping goes away) and print the counters */
void newport_arm_stop();

#endif
