/* mon.cc -- TCP monitor/console server for mips-axi.
 *
 * One plain-TCP listener (default port 2323, one client at a time). Two modes:
 *   console (default): raw passthrough. client byte -> SCC Rx FIFO (reg 0x3b,
 *     bit8 edge = push; poll reg 0x3a bit8 = scc_rx_full first). SCC Tx bytes
 *     reach the client via mon_console_out() called from read_char_fifo().
 *   monitor (Ctrl-] toggles): line-oriented commands that read the same control
 *     regs the exit --dump path uses -- FSM states, PC/EPC, GPRs, retire trace,
 *     and halt/step/go via the single-step control bits. Purely additive; nothing
 *     runs unless a client connects.
 *
 * The client `mipsmon` is a dumb raw-terminal pipe; this side owns the modes,
 * the echo, and the command table.
 */
#include "mon.hh"
#include "driver.hh"
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>

static Driver *g_d = nullptr;
static int g_listen = -1;
static int g_client = -1;
static bool g_monitor = false;          /* false = console passthrough, true = cmds */
static char g_line[256];
static int  g_linelen = 0;
static uint32_t g_cr = 8 | 2;           /* base control reg (matches main's cr) */

static void set_nonblock(int fd) {
  int fl = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

static void mon_send(const char *s) {
  if(g_client >= 0) {
    ssize_t r = write(g_client, s, strlen(s));
    (void)r;
  }
}

void mon_init(Driver *d, int port) {
  g_d = d;
  g_listen = socket(AF_INET, SOCK_STREAM, 0);
  if(g_listen < 0) {
    return;
  }
  int one = 1;
  setsockopt(g_listen, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  struct sockaddr_in a;
  memset(&a, 0, sizeof(a));
  a.sin_family = AF_INET;
  a.sin_addr.s_addr = INADDR_ANY;
  a.sin_port = htons(port);
  if(bind(g_listen, (struct sockaddr*)&a, sizeof(a)) < 0) {
    close(g_listen);
    g_listen = -1;
    return;
  }
  listen(g_listen, 1);
  set_nonblock(g_listen);
  printf("[mon] monitor/console on tcp:%d\n", port);
  std::fflush(nullptr);
}

/* push one byte into the SCC channel-A Rx FIFO (see ip_hdl S00_AXI:
 * scc_rx_data = slv_reg59[7:0], scc_rx_push = bit8 rising edge). */
static void scc_rx_push(uint8_t c) {
  for(int i = 0; i < 4096; i++) {
    if((g_d->read32(0x3a) & (1u << 8)) == 0) {   /* not full */
      break;
    }
  }
  g_d->write32(0x3b, 0x100u | c);
  g_d->write32(0x3b, 0);
}

void mon_console_out(int c) {
  if(g_client >= 0 && !g_monitor) {
    char b = (char)c;
    ssize_t r = write(g_client, &b, 1);
    (void)r;
  }
}

static const char *g_help =
  "monitor: s(tate) pc epc regs r<N> trace[N] halt go step[N] ret help  c/empty=console\r\n";

static void mon_cmd(char *line) {
  Driver *d = g_d;
  char out[1024];
  while(*line == ' ') {
    line++;
  }
  if(line[0] == 0 || (line[0] == 'c' && line[1] == 0)) {
    g_monitor = false;
    mon_send("\r\n[console -- Ctrl-] for monitor]\r\n");
    return;
  }
  else if(!strncmp(line, "help", 4) || line[0] == '?') {
    mon_send(g_help);
  }
  else if(!strncmp(line, "state", 5) || (line[0] == 's' && line[1] != 't')) {
    uint32_t st = d->read32(0xd);
    snprintf(out, sizeof(out),
      "core=%u l2=%u l1i=%u l1d=%u axi=%u inflight=%u l2rsp=%u\r\n",
      st & 31, (st >> 5) & 15, (st >> 9) & 7, (st >> 12) & 15,
      (st >> 16) & 15, (st >> 20) & 63, (st >> 26) & 15);
    mon_send(out);
  }
  else if(!strncmp(line, "pc", 2)) {
    snprintf(out, sizeof(out), "pc=%08x last_addr=%08x icnt=%u\r\n",
             d->read32(7), d->read32(9), d->read32(0));
    mon_send(out);
  }
  else if(!strncmp(line, "epc", 3)) {
    snprintf(out, sizeof(out), "epc=%08x badvaddr=%08x cause=%u\r\n",
             d->read32(0xb), d->read32(0xc), d->read32(0x26) & 31);
    mon_send(out);
  }
  else if(!strncmp(line, "ret", 3)) {
    snprintf(out, sizeof(out), "retired=%u\r\n", d->read32(0x16));
    mon_send(out);
  }
  else if(!strncmp(line, "regs", 4) || (line[0] == 'r' && line[1] != 'e')) {
    /* "r N" = single GPR, "regs"/"r" = all 32 */
    const char *p = line + 1;
    while(*p == ' ') {
      p++;
    }
    if(*p >= '0' && *p <= '9') {
      int n = atoi(p) & 31;
      d->write32(14, n);
      snprintf(out, sizeof(out), "r%d=%08x\r\n", n, d->read32(0xe));
      mon_send(out);
    }
    else {
      for(int n = 0; n < 32; n++) {
        d->write32(14, n);
        snprintf(out, sizeof(out), "r%-2d=%08x%s", n, d->read32(0xe),
                 (n & 3) == 3 ? "\r\n" : " ");
        mon_send(out);
      }
      mon_send("\r\n");
    }
  }
  else if(!strncmp(line, "trace", 5) || line[0] == 't') {
    const char *p = line + (line[0] == 't' && line[1] == 'r' ? 5 : 1);
    while(*p == ' ') {
      p++;
    }
    int want = (*p >= '0' && *p <= '9') ? atoi(p) : 16;
    uint32_t n = d->read32(0x16);           /* retired count = trace depth */
    uint32_t lo = (n > (uint32_t)want) ? n - want : 0;
    for(uint32_t i = lo; i <= n; i++) {
      d->write32(0x16, i);
      snprintf(out, sizeof(out), "%u: %08x\r\n", i, d->read32(0x17));
      mon_send(out);
    }
  }
  else if(!strncmp(line, "halt", 4) || line[0] == 'h') {
    g_cr |= (1u << 31);                     /* single-step mode = freeze */
    d->write32(4, g_cr);
    mon_send("[halted]\r\n");
  }
  else if(!strncmp(line, "go", 2) || line[0] == 'g') {
    g_cr &= ~(1u << 31);
    d->write32(4, g_cr);
    mon_send("[running]\r\n");
  }
  else if(!strncmp(line, "step", 4) || line[0] == 'n') {
    const char *p = line + (line[0] == 's' ? 4 : 1);
    while(*p == ' ') {
      p++;
    }
    int cnt = (*p >= '0' && *p <= '9') ? atoi(p) : 1;
    d->write32(4, g_cr | (1u << 31));       /* ensure step mode */
    for(int i = 0; i < cnt; i++) {
      d->write32(4, g_cr | (1u << 31) | (1u << 30));   /* 0->1 pulse = step */
      d->write32(4, g_cr | (1u << 31));
    }
    snprintf(out, sizeof(out), "[stepped %d] pc=%08x\r\n", cnt, d->read32(7));
    mon_send(out);
  }
  else {
    mon_send("? (help)\r\n");
  }
  mon_send("mips> ");
}

void mon_poll(void) {
  if(g_listen < 0) {
    return;
  }
  if(g_client < 0) {
    int fd = accept(g_listen, nullptr, nullptr);
    if(fd < 0) {
      return;
    }
    g_client = fd;
    set_nonblock(fd);
    g_monitor = false;
    g_linelen = 0;
    mon_send("\r\n[mips-axi console] Ctrl-] = monitor toggle, Ctrl-\\ = quit (client)\r\n");
    return;
  }
  uint8_t rb[256];
  ssize_t n = read(g_client, rb, sizeof(rb));
  if(n == 0) {
    close(g_client);
    g_client = -1;
    return;
  }
  if(n < 0) {
    return;                                 /* EWOULDBLOCK */
  }
  for(ssize_t i = 0; i < n; i++) {
    uint8_t c = rb[i];
    if(c == 0x1d) {                         /* Ctrl-] : toggle mode */
      g_monitor = !g_monitor;
      if(g_monitor) {
        g_linelen = 0;
        mon_send("\r\n[monitor -- 'help']\r\nmips> ");
      }
      else {
        mon_send("\r\n[console]\r\n");
      }
      continue;
    }
    if(!g_monitor) {
      scc_rx_push(c);
    }
    else {
      if(c == '\r' || c == '\n') {
        mon_send("\r\n");
        g_line[g_linelen] = 0;
        mon_cmd(g_line);
        g_linelen = 0;
      }
      else if(c == 0x7f || c == 8) {        /* backspace */
        if(g_linelen > 0) {
          g_linelen--;
          mon_send("\b \b");
        }
      }
      else if(g_linelen < (int)sizeof(g_line) - 1) {
        g_line[g_linelen++] = (char)c;
        char e[2] = { (char)c, 0 };
        mon_send(e);                        /* echo */
      }
    }
  }
}
