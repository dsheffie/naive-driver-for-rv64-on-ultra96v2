#include "chd_reader.hh"

#include <cstring>
#include <unistd.h>

extern "C" {
#include "libchdr/include/libchdr/chd.h"
}

chd_reader::~chd_reader() {
  if(chd) { chd_close(chd); chd = nullptr; }
  if(fp)  { fclose(fp);    fp  = nullptr; }   /* libchdr is a non-owner of the FILE* */
}

bool chd_reader::is_magic(const uint8_t *first8) {
  return memcmp(first8, "MComprHD", 8) == 0;
}

bool chd_reader::open(int fd) {
  /* dup so our fclose on close never touches the fd sgi_scsi still owns */
  int dfd = dup(fd);
  if(dfd < 0) { fprintf(stderr, "chd_reader: dup() failed\n"); return false; }
  fp = fdopen(dfd, "rb");
  if(!fp) { close(dfd); fprintf(stderr, "chd_reader: fdopen() failed\n"); return false; }

  chd_error e = chd_open_file(fp, CHD_OPEN_READ, nullptr, &chd);
  if(e != CHDERR_NONE) {
    fprintf(stderr, "chd_reader: chd_open_file failed: %s\n", chd_error_string(e));
    chd = nullptr; fclose(fp); fp = nullptr; return false;
  }

  const chd_header *h = chd_get_header(chd);
  hunkbytes   = h->hunkbytes;
  v_size      = h->logicalbytes;
  total_hunks = h->hunkcount ? h->hunkcount : h->totalhunks;
  if(hunkbytes == 0 || (hunkbytes % 512u) != 0) {
    fprintf(stderr, "chd_reader: unsupported hunkbytes %u (not a nonzero multiple of 512)\n", hunkbytes);
    chd_close(chd); chd = nullptr; fclose(fp); fp = nullptr; return false;
  }
  secs_per_hunk = hunkbytes / 512u;
  hunkbuf.resize(hunkbytes);
  cached_hunk = ~0ULL;

  fprintf(stderr, "chd_reader: CHD v%u, %llu bytes, hunk=%u (%u x 512), %llu hunks\n",
          h->version, (unsigned long long)v_size, hunkbytes, secs_per_hunk,
          (unsigned long long)total_hunks);
  return true;
}

void chd_reader::read_block(uint64_t lba, uint8_t *dst) {
  const uint64_t hunknum = lba / secs_per_hunk;
  const uint32_t off     = (uint32_t)(lba % secs_per_hunk) * 512u;
  if(hunknum >= total_hunks) { memset(dst, 0, 512); return; }
  if(hunknum != cached_hunk) {
    chd_error e = chd_read(chd, (uint32_t)hunknum, hunkbuf.data());
    if(e != CHDERR_NONE) {
      fprintf(stderr, "chd_reader: chd_read hunk %llu failed: %s\n",
              (unsigned long long)hunknum, chd_error_string(e));
      memset(dst, 0, 512); cached_hunk = ~0ULL; return;
    }
    cached_hunk = hunknum;
  }
  memcpy(dst, hunkbuf.data() + off, 512);
}
