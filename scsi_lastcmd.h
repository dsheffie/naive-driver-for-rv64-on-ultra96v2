#ifndef __SCSI_LASTCMD_H__
#define __SCSI_LASTCMD_H__
#include <cstdint>

/* Snapshot of the most-recently-serviced SCSI command, filled by scsi_arm and
 * dumped by the TCP monitor ("dump scsi" / "scsi").  Both live in the mips-axi
 * process, so this is a plain shared global -- no AXI/RTL involvement.  The
 * headline diagnostic is moved vs bufsz: a shortfall == a multi-segment HPC3
 * DMA the shim only partly delivered (the bug we chased). */
struct scsi_lastcmd_t {
  uint32_t n_cmds;                 /* total commands serviced this boot */
  uint32_t seq;                    /* doorbell sequence of this command */
  uint8_t  cdb[12];                /* raw CDB bytes */
  uint8_t  dest, lun, to_device, valid;
  uint64_t wr_lba;                 /* WRITE start LBA (else 0) */
  uint32_t bufsz, moved, residual; /* disk bytes / DMA'd bytes / shortfall */
  uint8_t  scsi_status, tgt_status;
  uint32_t nbdp;                   /* HPC3 descriptor-chain head */
  uint32_t n_desc;                 /* descriptors walked (chain length) */
  struct { uint32_t bp, count, next, eox; } desc[16];
};
extern scsi_lastcmd_t g_last_scsi;

#endif
