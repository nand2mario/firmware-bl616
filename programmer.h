#ifndef __PROGRAMMER_H__
#define __PROGRAMMER_H__

#include <stdint.h>
#include <stdbool.h>

#define JTAG_MAX_CHAIN 8

// Tang Primer 25K
#define IDCODE_GW5A_25 0x0001281b
// Tang Mega/Console 60K
#define IDCODE_GW5AT_60 0x0001481b
// Tang Mega/Console/Mega Pro 138K
#define IDCODE_GWAST_138 0x0001081b
// Not a current Tang board
#define IDCODE_GW5AT_138 0x0001181b
// Tang Nano 20K
#define IDCODE_GW2A_18 0x0000081b

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

// for fast programming
extern void jtag_writeTDI_msb_first_gpio_out_mode(const uint8_t *tx, int bytes, bool end);
extern void jtag_enter_gpio_out_mode();
extern void jtag_exit_gpio_out_mode();

#endif