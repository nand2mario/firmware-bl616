#ifndef PROGRAMMER_H
#define PROGRAMMER_H

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// main function: program FPGA with a bitstream (.bin) file
// return true if successful
extern bool fpga_program(const char *fname);

#ifdef __cplusplus
}
#endif

#endif // PROGRAMMER_H