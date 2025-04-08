/*
 * GPIO JTAG programmer for Gowin GW5A and GW2A
 *
 * (c) 2025.2, nand2mario
 *
 * This source code is licensed under the Apache 2.0 license found in the
 * LICENSE file in the root directory of this source tree. 
 *
 * Based in part on openFPGALoader by Gwenhael Goavec-Merou
 */

#include <string.h>
#include "programmer.h"
#include "utils.h"
#include <FreeRTOS.h>
#include "task.h"

int chain_len;
uint32_t idcodes[JTAG_MAX_CHAIN];

#define JTAG_TMS_BUFFER_SIZE 128

typedef enum tapState_t {
    TEST_LOGIC_RESET = 0,
    RUN_TEST_IDLE = 1,
    SELECT_DR_SCAN = 2,
    CAPTURE_DR = 3,
    SHIFT_DR = 4,
    EXIT1_DR = 5,
    PAUSE_DR = 6,
    EXIT2_DR = 7,
    UPDATE_DR = 8,
    SELECT_IR_SCAN = 9,
    CAPTURE_IR = 10,
    SHIFT_IR = 11,
    EXIT1_IR = 12,
    PAUSE_IR = 13,
    EXIT2_IR = 14,
    UPDATE_IR = 15,
    UNKNOWN = 16,
} tapState_t;

static int _num_tms;                // in bits
static int _curr_tms, _curr_tdi;
static unsigned char _tms_buffer[JTAG_TMS_BUFFER_SIZE];
static tapState_t _state;

const int JTAG_RATE=5000000;						// 5Mhz jtag speed
const int JTAG_DELAY=32;                			// 320Mhz / 5Mhz / 2 = 32 cycles

// ------------------------------------------------------------
// Physical I/O functions
// follow libgpiodJtagBitbang.cpp
#define get_mcycle() (0)
#define wait_mcycle(start, cycles) (0)
// static inline uint32_t get_mcycle(void) {
// 	uint32_t mcycle;
// 	asm volatile("csrr %0, mcycle" : "=r"(mcycle));
// 	return mcycle;
// }
// static inline uint32_t wait_mcycle(uint32_t start, uint32_t cycles) {
//     uint32_t end = start + cycles;
//     uint32_t now;
//     while ((now = get_mcycle()) < end);
//     return now;
// }

static int jtag_read_tdo() {
    return GPIO_PIN_JTAG_TDO_V();
}

static int jtag_writeTMS(const uint8_t *tms_buf, uint32_t len, bool flush_buffer, const uint8_t tdi) {
	if (len == 0) // nothing -> stop
		return len;

	// uint32_t start = get_mcycle();
	for (uint32_t i = 0; i < len; i++) {
		int tms = ((tms_buf[i >> 3] & (1 << (i & 7))) ? 1 : 0);

        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L(); // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_L();                                      // low  TCK
		// start = wait_mcycle(start, JTAG_DELAY);

        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L(); // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_H();                                      // high TCK

        _curr_tms = tms;
		// start = wait_mcycle(start, JTAG_DELAY);
	}
    // GPIO_PIN_JTAG_TCK_L();                                          // low  TCK
	_curr_tdi = tdi;

	return len;
}

uint64_t jtag_writetdi_time = 0;	// performance counter

//         TMS  TCK  TDO  TDI 
// Nano20K: 16   10   14   12
// Others:   0    1    2    3
// bit 25 is set, bit 26 is clear
#ifdef TANG_NANO20K
volatile uint32_t *reg_gpio_tms = (volatile uint32_t *)0x20000904;     // gpio16
volatile uint32_t *reg_gpio_tck = (volatile uint32_t *)0x200008ec;     // gpio10
volatile uint32_t *reg_gpio_tdo = (volatile uint32_t *)0x200008fc;	   // gpio14
volatile uint32_t *reg_gpio_tdi = (volatile uint32_t *)0x200008f4;     // gpio12
#else
volatile uint32_t *reg_gpio_tms = (volatile uint32_t *)0x200008c4;     // gpio0, bl616 reference 4.8.5
volatile uint32_t *reg_gpio_tck = (volatile uint32_t *)0x200008c8;     // gpio1
volatile uint32_t *reg_gpio_tdo = (volatile uint32_t *)0x200008cc;	   // gpio2
volatile uint32_t *reg_gpio_tdi = (volatile uint32_t *)0x200008d0;     // gpio3
#endif

// nand2mario: this is the hotspot for JTAG programming and optimized for performance
static int jtag_writeTDI(const uint8_t *tx, uint8_t *rx, int len, bool end) {
	bool tms = _curr_tms;
	bool tdi = _curr_tdi;
	const uint32_t mask_set = (1 << 25);
	const uint32_t mask_clear = (1 << 26);

	uint64_t time_start = bflb_mtimer_get_time_us();

	if (rx)
		memset(rx, 0, len / 8);

	// uint32_t start = get_mcycle();
	for (uint32_t i = 0; i < len; i++) {
		if (end && (i == len - 1)) {
			tms = 1;
			*reg_gpio_tms |= mask_set;
		}

		if (tx)
			tdi = (tx[i >> 3] & (1 << (i & 7)));

		*reg_gpio_tdi |= tdi ? mask_set : mask_clear;	// set TDI
		*reg_gpio_tck |= mask_clear;					// clock low
		*reg_gpio_tck |= mask_set;						// clock high

		if (rx) {
			if (jtag_read_tdo() > 0)
				rx[i >> 3] |= 1 << (i & 7);
		}
	}
	_curr_tms = tms;
	_curr_tdi = tdi;

	jtag_writetdi_time += bflb_mtimer_get_time_us() - time_start;
	return len;
}

#define GPIO_INT_MASK    (1<<22)
#define GPIO_FUNC_SWGPIO (0xB<<8)
#define GPIO_OUTPUT_EN   (1<<6)
#define GPIO_INPUT_EN    (1)
#define GPIO_SCHMITT_EN  (1<<1)
#define GPIO_DRV_3       (3<<2)
#define GPIO_MODE_VALUE  (0<<30)
#define GPIO_MODE_SETCLR (1<<30)
#define GPIO_MODE_DMA    (2<<30)
#define GPIO_MODE_DMA_SETCLR (3<<30)
uint32_t jtag_tms_cfg, jtag_tck_cfg, jtag_tdi_cfg;
volatile uint32_t *reg_gpio0_31 = (volatile uint32_t *)0x20000ae4;  // gpio_cfg136，Register Controlled GPIO Output Value

// set GPIO0 (TMS), GPIO1 (TCK) and GPIO3 (TDI) as direct output mode
void jtag_enter_gpio_out_mode() {
	jtag_tms_cfg = *reg_gpio_tms;
	jtag_tck_cfg = *reg_gpio_tck;
	jtag_tdi_cfg = *reg_gpio_tdi;
	*reg_gpio_tms = GPIO_INT_MASK | GPIO_FUNC_SWGPIO | GPIO_OUTPUT_EN | GPIO_SCHMITT_EN | GPIO_DRV_3;
	*reg_gpio_tck = GPIO_INT_MASK | GPIO_FUNC_SWGPIO | GPIO_OUTPUT_EN | GPIO_SCHMITT_EN | GPIO_DRV_3;
	*reg_gpio_tdi = GPIO_INT_MASK | GPIO_FUNC_SWGPIO | GPIO_OUTPUT_EN | GPIO_SCHMITT_EN | GPIO_DRV_3;
}

// restore GPIO1 and GPIO3 settings
void jtag_exit_gpio_out_mode() {
	*reg_gpio_tms = jtag_tms_cfg;
	*reg_gpio_tck = jtag_tck_cfg;
	*reg_gpio_tdi = jtag_tdi_cfg;
}

// nand2mario: this is faster than jtag_writeTDI()
// 1. avoid data conversion by writing *tx MSB first
// 2. use GPIO_CFG144 to set GPIO0-3 as output
void jtag_writeTDI_msb_first_gpio_out_mode(const uint8_t *tx, int bytes, bool end) {
	for (int i = 0; i < bytes; i++) {
		uint8_t byte = tx[i];
#ifdef TANG_NANO20K
		// bit 7
		*reg_gpio0_31 = (byte & 0x80) << 5;                // bit 12 (TDI) = data, bit 10 (TCK) = 0
		*reg_gpio0_31 = ((byte & 0x80) << 5) | (1 << 10);  // bit 12 (TDI) = data, bit 10 (TCK) = 1
		// bit 6
		*reg_gpio0_31 = (byte & 0x40) << 6; 
		*reg_gpio0_31 = ((byte & 0x40) << 6) | (1 << 10); 
		// bit 5
		*reg_gpio0_31 = (byte & 0x20) << 7; 
		*reg_gpio0_31 = ((byte & 0x20) << 7) | (1 << 10); 
		// bit 4
		*reg_gpio0_31 = (byte & 0x10) << 8; 
		*reg_gpio0_31 = ((byte & 0x10) << 8) | (1 << 10); 
		// bit 3
		*reg_gpio0_31 = (byte & 0x8) << 9; 
		*reg_gpio0_31 = ((byte & 0x8) << 9) | (1 << 10); 
		// bit 2
		*reg_gpio0_31 = (byte & 0x4) << 10; 
		*reg_gpio0_31 = ((byte & 0x4) << 10) | (1 << 10); 
		// bit 1
		*reg_gpio0_31 = (byte & 0x2) << 11; 
		*reg_gpio0_31 = ((byte & 0x2) << 11) | (1 << 10); 
		// bit 0
		*reg_gpio0_31 = (byte & 0x1) << 12 | (i == bytes-1 && end ? (1 << 16) : 0); 	// bit 16: TMS
		*reg_gpio0_31 = ((byte & 0x1) << 12) | (1 << 10) | (i == bytes-1 && end ? (1 << 16) : 0); 
#else
		// bit 7
		*reg_gpio0_31 = (byte & 0x80) >> 4;           // bit 3 (TDI) = data, bit 1 (TCK) = 0
		*reg_gpio0_31 = ((byte & 0x80) >> 4) | 2;     // bit 3 (TDI) = data, bit 1 (TCK) = 1
		// bit 6
		*reg_gpio0_31 = (byte & 0x40) >> 3;     
		*reg_gpio0_31 = ((byte & 0x40) >> 3) | 2; 
		// bit 5
		*reg_gpio0_31 = (byte & 0x20) >> 2;     
		*reg_gpio0_31 = ((byte & 0x20) >> 2) | 2; 
		// bit 4
		*reg_gpio0_31 = (byte & 0x10) >> 1;     
		*reg_gpio0_31 = ((byte & 0x10) >> 1) | 2; 
		// bit 3
		*reg_gpio0_31 = (byte & 0x8);     
		*reg_gpio0_31 = (byte & 0x8) | 2; 
		// bit 2
		*reg_gpio0_31 = (byte & 0x4) << 1;     
		*reg_gpio0_31 = ((byte & 0x4) << 1) | 2; 
		// bit 1
		*reg_gpio0_31 = (byte & 0x2) << 2;     
		*reg_gpio0_31 = ((byte & 0x2) << 2) | 2; 
		// bit 0
		*reg_gpio0_31 = ((byte & 0x1) << 3) | (i == bytes-1 && end);  	// TMS=1 if at the end
		*reg_gpio0_31 = ((byte & 0x1) << 3) | 2 | (i == bytes-1 && end);	// TMS=1 if at the end
#endif
	}
	_curr_tms = end;
}

/*
static int jtag_writeTDI_slow(const uint8_t *tx, uint8_t *rx, int len, bool end) {
	int tms = _curr_tms;
	int tdi = _curr_tdi;

	uint64_t time_start = bflb_mtimer_get_time_us();

	if (rx)
		memset(rx, 0, len / 8);

	// uint32_t start = get_mcycle();
	for (uint32_t i = 0; i < len; i++) {
		if (end && (i == len - 1))
			tms = 1;

		if (tx)
			tdi = (tx[i >> 3] & (1 << (i & 7))) ? 1 : 0;

        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L();     // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_L();                                          // low  TCK 
		// start = wait_mcycle(start, JTAG_DELAY);

        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L();     // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_H();                                          // high TCK

        _curr_tms = tms;
        _curr_tdi = tdi;

		if (rx) {
			if (jtag_read_tdo() > 0)
				rx[i >> 3] |= 1 << (i & 7);
		}
		// start = wait_mcycle(start, JTAG_DELAY);
	}
    // GPIO_PIN_JTAG_TCK_L();                                          // low  TCK

	jtag_writetdi_time += bflb_mtimer_get_time_us() - time_start;
	return len;
}
*/

static int flushTMS(bool flush_buffer) {
	int ret = 0;
	if (_num_tms != 0) {
		ret = jtag_writeTMS(_tms_buffer, _num_tms, flush_buffer, _curr_tdi);

		/* reset buffer and number of bits */
		memset(_tms_buffer, 0, JTAG_TMS_BUFFER_SIZE);
		_num_tms = 0;
	} else if (flush_buffer) {
		// jtag_flush();
	}
	return ret;    
}

static int jtag_toggleClk_(uint8_t tms, uint8_t tdi, uint32_t clk_len)
{
	// uint32_t start = get_mcycle();	
	for (uint32_t i = 0; i < clk_len; i++) {
        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L();     // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_L();                                          // low  TCK
		// start = wait_mcycle(start, JTAG_DELAY);

        if (tms) GPIO_PIN_JTAG_TMS_H(); else GPIO_PIN_JTAG_TMS_L();     // send TMS
        if (tdi) GPIO_PIN_JTAG_TDI_H(); else GPIO_PIN_JTAG_TDI_L();     // send TDI
        GPIO_PIN_JTAG_TCK_H();                                          // high TCK
		// start = wait_mcycle(start, JTAG_DELAY);
    }
	_curr_tms = tms;
	_curr_tdi = tdi;
	
    return clk_len;
}

static void jtag_toggleClk(int nb)
{
	unsigned char c = (TEST_LOGIC_RESET == _state) ? 1 : 0;
	flushTMS(false);
	jtag_toggleClk_(c, 0, nb);
}

// isFull() and flush() is moot

// ------------------------------------------------------------
// JTAG functions, similar to jtag.cpp
static unsigned _dr_bits_before, _dr_bits_after;
static uint8_t _dr_bits[4];     // nand2mario: shiftDR() buffer. in our case, there's only one device.
                                // so shiftDr() is actually not needed.
static unsigned _ir_bits_before, _ir_bits_after;
static uint8_t _ir_bits[4];
                                
static void setTMS(unsigned char tms) {
	if (_num_tms+1 == JTAG_TMS_BUFFER_SIZE * 8)
		flushTMS(false);
	if (tms != 0)
		_tms_buffer[_num_tms>>3] |= (0x1) << (_num_tms & 0x7);
	_num_tms++;
}

static void go_test_logic_reset() {
    for (int i = 0; i < 6; i++)
        setTMS(0x01);
    flushTMS(false);
    _state = TEST_LOGIC_RESET;
}

static void set_state(tapState_t newState) {
    const uint8_t tdi = 1;
	_curr_tdi = tdi;
	unsigned char tms = 0;
	while (newState != _state) {
		// display("_state : %16s(%02d) -> %s(%02d) ",
		// 	getStateName((tapState_t)_state),
		// 	_state,
		// 	getStateName((tapState_t)newState), newState);
		switch (_state) {
		case TEST_LOGIC_RESET:
			if (newState == TEST_LOGIC_RESET) {
				tms = 1;
			} else {
				tms = 0;
				_state = RUN_TEST_IDLE;
			}
			break;
		case RUN_TEST_IDLE:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
		case SELECT_DR_SCAN:
			switch (newState) {
			case CAPTURE_DR:
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
			case EXIT2_DR:
			case UPDATE_DR:
				tms = 0;
				_state = CAPTURE_DR;
				break;
			default:
				tms = 1;
				_state = SELECT_IR_SCAN;
			}
			break;
		case SELECT_IR_SCAN:
			switch (newState) {
			case CAPTURE_IR:
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
			case EXIT2_IR:
			case UPDATE_IR:
				tms = 0;
				_state = CAPTURE_IR;
				break;
			default:
				tms = 1;
				_state = TEST_LOGIC_RESET;
			}
			break;
			/* DR column */
		case CAPTURE_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
				_state = SHIFT_DR;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case SHIFT_DR:
			if (newState == SHIFT_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_DR;
			}
			break;
		case EXIT1_DR:
			switch (newState) {
			case PAUSE_DR:
			case EXIT2_DR:
			case SHIFT_DR:
			case EXIT1_DR:
				tms = 0;
				_state = PAUSE_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case PAUSE_DR:
			if (newState == PAUSE_DR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_DR;
			}
			break;
		case EXIT2_DR:
			switch (newState) {
			case SHIFT_DR:
			case EXIT1_DR:
			case PAUSE_DR:
				tms = 0;
				_state = SHIFT_DR;
				break;
			default:
				tms = 1;
				_state = UPDATE_DR;
			}
			break;
		case UPDATE_DR:
		case UPDATE_IR:
			if (newState == RUN_TEST_IDLE) {
				tms = 0;
				_state = RUN_TEST_IDLE;
			} else {
				tms = 1;
				_state = SELECT_DR_SCAN;
			}
			break;
			/* IR column */
		case CAPTURE_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
				_state = SHIFT_IR;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case SHIFT_IR:
			if (newState == SHIFT_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT1_IR;
			}
			break;
		case EXIT1_IR:
			switch (newState) {
			case PAUSE_IR:
			case EXIT2_IR:
			case SHIFT_IR:
			case EXIT1_IR:
				tms = 0;
				_state = PAUSE_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		case PAUSE_IR:
			if (newState == PAUSE_IR) {
				tms = 0;
			} else {
				tms = 1;
				_state = EXIT2_IR;
			}
			break;
		case EXIT2_IR:
			switch (newState) {
			case SHIFT_IR:
			case EXIT1_IR:
			case PAUSE_IR:
				tms = 0;
				_state = SHIFT_IR;
				break;
			default:
				tms = 1;
				_state = UPDATE_IR;
			}
			break;
		case UNKNOWN:;
			// UNKNOWN should not be valid...
			// throw std::exception();
		}

		setTMS(tms);
		// display("%d %d %d %x\n", tms, _num_tms-1, _state,
		// 	_tms_buffer[(_num_tms-1) / 8]);
	}
	/* force write buffer */
	flushTMS(false);
}

static int read_write(const uint8_t *tdi, unsigned char *tdo, int len, char last) {
    flushTMS(false);
    jtag_writeTDI(tdi, tdo, len, last);
    if (last == 1)
        _state = (_state == SHIFT_DR) ? EXIT1_DR : EXIT1_IR;
    return 0;
}

static int shiftIR_end(const unsigned char *tdi, unsigned char *tdo, int irlen, tapState_t end_state)
{
    // display("%s: avant shiftIR\n", __func__);

	/* if not in SHIFT IR move to this state */
	if (_state != SHIFT_IR) {
		set_state(SHIFT_IR);
		if (_ir_bits_before)
			read_write(_ir_bits, NULL, _ir_bits_before, false);
	}

	// display("%s: envoi ircode\n", __func__);

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	read_write(tdi, tdo, irlen, _ir_bits_after == 0 && end_state != SHIFT_IR);

	/* it's asked to move out of SHIFT IR state */
	if (end_state != SHIFT_IR) {
		/* again if devices after fill '1' */
		if (_ir_bits_after > 0)
			read_write(_ir_bits, NULL, _ir_bits_after, true);
		/* move to the requested state */
		set_state(end_state);
	}

	return 0;
}
static int shiftIR(const uint8_t *tdi, unsigned char *tdo, int irlen) {
    shiftIR_end(tdi, tdo, irlen, RUN_TEST_IDLE);
	return 0;
}

static int shiftDR_end(const uint8_t *tdi, unsigned char *tdo, int drlen, tapState_t end_state)
{
	/* if current state not shift DR
	 * move to this state
	 */
	if (_state != SHIFT_DR) {
		set_state(SHIFT_DR);
		flushTMS(false);  // force transmit tms state

		if (_dr_bits_before) {
			printf("dr_bits_before: %d\n", _dr_bits_before);
			read_write(_dr_bits, NULL, _dr_bits_before, false);
		}
	}

	/* write tdi (and read tdo) to the selected device
	 * end (ie TMS high) is used only when current device
	 * is the last of the chain and a state change must
	 * be done
	 */
	read_write(tdi, tdo, drlen, _dr_bits_after == 0 && end_state != SHIFT_DR);

	/* if it's asked to move in FSM */
	if (end_state != SHIFT_DR) {
		/* if current device is not the last */
		if (_dr_bits_after) {
			printf("dr_bits_after: %d\n", _dr_bits_after);
			read_write(_dr_bits, NULL, _dr_bits_after, true);  // its the last force
								   // tms high with last bit
		}

		/* move to end_state */
		set_state(end_state);
	}
	return 0;
}

static int shiftDR(const uint8_t *tdi, unsigned char *tdo, int drlen) {
    shiftDR_end(tdi, tdo, drlen, RUN_TEST_IDLE);
	return 0;
}

// ------------------------------------------------------------
// Gowin specific: gowin.cpp
#define NOOP				0x02
#define ERASE_SRAM			0x05
#define XFER_DONE			0x09
#define READ_IDCODE			0x11
#define INIT_ADDR			0x12
#define READ_USERCODE		0x13
#define CONFIG_ENABLE		0x15
#define XFER_WRITE			0x17
#define CONFIG_DISABLE		0x3A
#define RELOAD				0x3C
#define STATUS_REGISTER		0x41

#define STATUS_CRC_ERROR			(1 << 0)
#define STATUS_BAD_COMMAND		(1 << 1)
#define STATUS_ID_VERIFY_FAILED	(1 << 2)
#define STATUS_TIMEOUT			(1 << 3)
#define STATUS_MEMORY_ERASE		(1 << 5)
#define STATUS_PREAMBLE			(1 << 6)
#define STATUS_SYSTEM_EDIT_MODE	(1 << 7)
#define STATUS_PRG_SPIFLASH_DIRECT (1 << 8)
#define STATUS_NON_JTAG_CNF_ACTIVE (1 << 10)
#define STATUS_BYPASS				(1 << 11)
#define STATUS_GOWIN_VLD			(1 << 12)
#define STATUS_DONE_FINAL			(1 << 13)
#define STATUS_SECURITY_FINAL		(1 << 14)
#define STATUS_READY				(1 << 15)
#define STATUS_POR				(1 << 16)
#define STATUS_FLASH_LOCK			(1 << 17)

typedef enum prog_mode {
    NONE_MODE = 0,
    SPI_MODE = 1,
    FLASH_MODE = 1,
    MEM_MODE = 2,
    READ_MODE = 3,
} prog_mode;

// static prog_mode _mode = NONE_MODE;
// uint16_t _checksum;

// trust we are little-endian
#define htole32(x) (x)
#define le32toh(x) (x)

bool send_command(uint8_t cmd)
{
	shiftIR(&cmd, NULL, 8);
	jtag_toggleClk(6);
	return true;
}

static uint32_t readReg32(uint8_t cmd)
{
	uint32_t reg = 0, tmp = 0xffffffffU;
	send_command(cmd);
	shiftDR((uint8_t *)&tmp, (uint8_t *)&reg, 32);
	return le32toh(reg);
}

static uint32_t idCode()
{
	return readReg32(READ_IDCODE);
}

static uint32_t readStatusReg()
{
	return readReg32(STATUS_REGISTER);	// 0x41
}

static uint32_t readUserCode()
{
	return readReg32(READ_USERCODE);	// 0x13
}

bool pollFlag(uint32_t mask, uint32_t value)
{
	uint32_t status;
	int timeout = 0;
	do {
		status = readStatusReg();
		if (timeout == 100000000){
			printf("timeout\r\n");
			return false;
		}
		timeout++;
		// vTaskDelay(1);
	} while ((status & mask) != value);

	return true;
}

bool enableCfg()
{
	send_command(CONFIG_ENABLE);	// 0x15
	return pollFlag(STATUS_SYSTEM_EDIT_MODE, STATUS_SYSTEM_EDIT_MODE);
}

bool disableCfg()
{
	send_command(CONFIG_DISABLE);	// 0x3A
	send_command(NOOP);				// 0x02
	return pollFlag(STATUS_SYSTEM_EDIT_MODE, 0);
}

void sendClkUs(unsigned us)
{
	uint64_t clocks = 15000000;     // very rough estimate: 15Mhz
	clocks *= us;
	clocks /= 1000000;
	jtag_toggleClk(clocks);
}



// ------------------------------------------------------------
// Public functions

bool is_gw2a;

int detectChain(int max_dev) {
    // char message[256];
    uint8_t rx_buff[4];
    const uint8_t tx_buff[4] = {0xff, 0xff, 0xff, 0xff};
    uint32_t tmp;

    // cleanup
    chain_len = 0;
    go_test_logic_reset();
	set_state(SHIFT_DR);

	for (int i = 0; i < max_dev; ++i) {
		read_write(tx_buff, rx_buff, 32, 0);
		tmp = 0;
		for (int ii = 0; ii < 4; ++ii)
			tmp |= (rx_buff[ii] << (8 * ii));

        if (tmp == 0) {
            return -1;              // TDO is stuck at 0
        }

		if (tmp == 0xffffffff) {    // end of chain
			break;
		}

        idcodes[chain_len++] = tmp;
	}
	set_state(TEST_LOGIC_RESET);
	flushTMS(true);

	if (chain_len > 0 && idcodes[0] == IDCODE_GW2A_18) {
		is_gw2a = true;
	}

	return chain_len;
}

bool eraseSRAM() {
	uint32_t id = idCode();
	uint32_t status = readStatusReg();
	overlay_cursor(2, 0);
	overlay_printf("ID=%08x, status=%08x", id, status);

	bool auto_boot_2nd_fail = (status & (1 << 4)) == (1 << 4);
	bool is_timeout = (status & (1 << 3)) == (1 << 3);
	bool bad_cmd = (status & STATUS_BAD_COMMAND) == STATUS_BAD_COMMAND;
	if (is_timeout || auto_boot_2nd_fail || bad_cmd) {
		overlay_status("Erase: send 0x3F\n");
		send_command(CONFIG_ENABLE);
		send_command(0x3F);
		send_command(CONFIG_DISABLE);
		send_command(NOOP);
		send_command(READ_IDCODE);
		send_command(NOOP);
		jtag_toggleClk(125 * 8);
	}

	if (!enableCfg()) {
		overlay_status("FAIL to enable configuration\r\n");
		return false;
	}
	send_command(ERASE_SRAM);	// 0x05
	send_command(NOOP);			// 0x02

	/* TN653 specifies to wait for 4ms with
	 * clock generated but
	 * status register bit MEMORY_ERASE goes low when ERASE_SRAM
	 * is send and goes high after erase
	 * this check seems enough
	 */
	if (idcodes[0] == 0x0001081b) // seems required for GW5AST...
		sendClkUs(10000);
	overlay_status("Erase: pollFlag...");
	if (pollFlag(STATUS_MEMORY_ERASE, STATUS_MEMORY_ERASE)) {
        overlay_status("Erase: OK");
        // success
    } else {
		overlay_status("FAIL\r\n");
		return false;
	}

	send_command(XFER_DONE);	// 0x09
	send_command(NOOP);			// 0x02
	overlay_status("Erase: disableCfg...");
	if (!disableCfg()) {		// 0x3A		<---- HANG here
		overlay_status("FAIL\r\n");
		return false;
	}
	overlay_status("Erase: disableCfg done...");
	send_command(NOOP);			// 0x02
    overlay_status("Erase: status=0x%08x\r\n", readStatusReg());

	return true;    
}

bool writeSRAM_start() {
	overlay_status("Load SRAM\r\n");
	// uint32_t status = readStatusReg();
	// if (_verbose)
	// 	displayReadReg("before write sram", readStatusReg());
	// ProgressBar progress("Load SRAM", length, 50, _quiet);
	send_command(CONFIG_ENABLE); // config enable 0x15

	/* UG704 3.4.3 */
	send_command(INIT_ADDR); // address initialize 0x12
    // printf("Status after INIT_ADDR: 0x%08x\r\n", readStatusReg());

	/* 2.2.6.4 */
	send_command(XFER_WRITE); // transfer configuration data 0x17

	set_state(SHIFT_DR);

    // clear checksum
	// _checksum = 0;
    return true;
}

// call this repeatedly to send data in chunks
// NOTE: length is in BITS, and has to be multiple of 16, except the last block
bool writeSRAM_send(const uint8_t *data, uint32_t length, bool last) {
    /* 2.2.6.5 */
    shiftDR_end(data, NULL, length, last ? RUN_TEST_IDLE : SHIFT_DR);

    // update checksum by summing up 16-bit unsigned words
	// for (uint32_t i = 0; i < (length >> 3); i+=2)
	// 	_checksum += *(uint16_t *)(data + i);

    return true;
}

bool writeSRAM_end() {

    // printf("Status after XFER_WRITE: 0x%08x\r\n", readStatusReg());
    // send checksum
    // send_command(0x0a);
    // uint32_t checksum = 0xF12D;     // _checksum;
	// shiftDR((uint8_t *)&checksum, NULL, 32);
	// send_command(0x08);

    // printf("Status after CHECKSUM: 0x%08x\r\n", readStatusReg());

	send_command(CONFIG_DISABLE); // config disable 0x3A
	send_command(NOOP); // noop 0x02

	uint32_t usercode = readUserCode();
	uint32_t status_reg = readStatusReg();
    overlay_status("Usercode=0x%08x, status=0x%08x\r\n", usercode, status_reg);

	if (status_reg & STATUS_DONE_FINAL) {
		// printf("DONE\r\n");
		return true;
	} else {
		// printf("FAIL\r\n");
		return false;
	}
}

void fpgaStatus() {
    overlay_status("Status=%08x, User=%08x", readStatusReg(), readUserCode());
}

void fpgaReset() {
	send_command(RELOAD);
	send_command(NOOP);    
    set_state(RUN_TEST_IDLE);
    jtag_toggleClk(1000000);
}