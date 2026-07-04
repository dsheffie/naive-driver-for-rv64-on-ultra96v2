/* mipsmon.cc -- human client for the mips-axi TCP monitor/console.
 *
 *   mipsmon [host [port]]        (default: fpga.local 2323)
 *
 * A raw-terminal pipe: your keystrokes go to the core's SCC Rx, its console
 * comes back, so you can type at the IRIX/Linux prompt. Ctrl-] toggles the
 * server's line-monitor (state/pc/epc/regs/trace/halt/step/go). Ctrl-\ quits
 * this client (handled locally, never sent to the core).
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <sys/socket.h>
#include <netdb.h>

static struct termios g_saved;
static bool g_raw = false;

static void restore_term(void) {
  if(g_raw) {
    tcsetattr(0, TCSANOW, &g_saved);
    g_raw = false;
  }
}

int main(int argc, char **argv) {
  const char *host = (argc > 1) ? argv[1] : "fpga.local";
  const char *port = (argc > 2) ? argv[2] : "2323";

  struct addrinfo hints, *res = nullptr;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  if(getaddrinfo(host, port, &hints, &res) != 0) {
    fprintf(stderr, "mipsmon: cannot resolve %s\n", host);
    return 1;
  }
  int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
  if(s < 0 || connect(s, res->ai_addr, res->ai_addrlen) < 0) {
    fprintf(stderr, "mipsmon: cannot connect to %s:%s\n", host, port);
    return 1;
  }
  freeaddrinfo(res);
  fprintf(stderr, "mipsmon: connected to %s:%s  (Ctrl-] monitor, Ctrl-\\ quit)\r\n",
          host, port);

  if(tcgetattr(0, &g_saved) == 0) {
    struct termios raw = g_saved;
    cfmakeraw(&raw);
    tcsetattr(0, TCSANOW, &raw);
    g_raw = true;
    atexit(restore_term);
  }

  for(;;) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    FD_SET(s, &fds);
    if(select(s + 1, &fds, nullptr, nullptr, nullptr) < 0) {
      break;
    }
    if(FD_ISSET(0, &fds)) {
      char b[256];
      int n = read(0, b, sizeof(b));
      if(n <= 0) {
        break;
      }
      for(int i = 0; i < n; i++) {
        if(b[i] == 0x1c) {                  /* Ctrl-\ : quit locally */
          restore_term();
          fprintf(stderr, "\r\n[mipsmon quit]\r\n");
          return 0;
        }
      }
      if(write(s, b, n) <= 0) {
        break;
      }
    }
    if(FD_ISSET(s, &fds)) {
      char b[256];
      int n = read(s, b, sizeof(b));
      if(n <= 0) {
        fprintf(stderr, "\r\n[mipsmon: disconnected]\r\n");
        break;
      }
      if(write(1, b, n) <= 0) {
        break;
      }
    }
  }
  restore_term();
  return 0;
}
