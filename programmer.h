#ifndef __PROGRAMMER_H__
#define __PROGRAMMER_H__

#include <stdint.h>
#include <stdbool.h>

#define JTAG_MAX_CHAIN 8

#define IDCODE_GW5AT_60 0x0001481b
#define IDCODE_GWAST_138 0x0001081b

extern int chain_len;
extern uint32_t idcodes[];

// Results in chain_len and idcodes
extern int detectChain(int max_dev);

extern bool eraseSRAM();

// Returns true if successful
extern bool writeSRAM_start();

// NOTE: length is in BITS, and has to be multiples of 16 except the last block
// last: true if this is the last block
extern bool writeSRAM_send(const uint8_t *data, uint32_t length, bool last);

extern bool writeSRAM_end();

extern void fpgaStatus();

extern void fpgaReset();

#endif