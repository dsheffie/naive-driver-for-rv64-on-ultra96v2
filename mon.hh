#ifndef __MON_HH__
#define __MON_HH__
/* TCP monitor/console server for mips-axi. Plain TCP (no telnet protocol):
 * default console passthrough (client bytes -> SCC Rx, SCC Tx -> client);
 * Ctrl-] toggles a line-oriented monitor (state/pc/epc/regs/trace/halt/step/go). */
class Driver;
void mon_init(Driver *d, int port);   /* one-time: open the listener */
void mon_poll(void);                  /* call each loop iteration (non-blocking) */
void mon_console_out(int c);          /* feed one console (SCC Tx) byte to the client */
#endif
