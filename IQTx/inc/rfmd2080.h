#ifndef RFMD2080_H_
#define RFMD2080_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <linux/types.h>
#include <math.h>

#define RFMD2080_LF			(0x00)
#define RFMD2080_XO			(0x01)
#define RFMD2080_CAL_TIME	(0x02)
#define RFMD2080_VCO_CTRL	(0x03)
#define RFMD2080_CT_CAL1	(0x04)
#define RFMD2080_CT_CAL2	(0x05)
#define RFMD2080_PLL_CAL1	(0x06)
#define RFMD2080_PLL_CAL2	(0x07)
#define RFMD2080_VCO_AUTO	(0x08)
#define RFMD2080_PLL_CTRL	(0x09)
#define RFMD2080_PLL_BIAS	(0x0A)
#define RFMD2080_MIX_CONT	(0x0B)
#define RFMD2080_P1_FREQ1	(0x0C)
#define RFMD2080_P1_FREQ2	(0x0D)
#define RFMD2080_P1_FREQ3	(0x0E)
#define RFMD2080_P2_FREQ1	(0x0F)
#define RFMD2080_P2_FREQ2	(0x10)
#define RFMD2080_P2_FREQ3	(0x11)
#define RFMD2080_FN_CTRL	(0x12)
#define RFMD2080_EXT_MOD	(0x13)
#define RFMD2080_FMOD		(0x14)
#define RFMD2080_SDI_CTRL	(0x15)
#define RFMD2080_GPO		(0x16)
#define RFMD2080_T_VCO		(0x17)
#define RFMD2080_IQMOD1		(0x18)
#define RFMD2080_IQMOD2		(0x19)
#define RFMD2080_IQMOD3		(0x1A)
#define RFMD2080_IQMOD4		(0x1B)
#define RFMD2080_T_CTRL		(0x1C)
#define RFMD2080_DEV_CTRL	(0x1D)
#define RFMD2080_TEST		(0x1E)
#define RFMD2080_READBACK	(0x1F)

#define GPIO0_START			0x44E07000
#define GPIO0_END 			0x44E07FFF
#define GPIO0_SIZE 			(GPIO0_END - GPIO0_START)
#define GPIO_OE 			0x134
#define GPIO_SETDATAOUT 	0x194
#define GPIO_CLEARDATAOUT 	0x190
#define GPIO_DEBOUNCETIME	0x154
#define GPIO_DATAIN 		0x138

#define BIT_TIME	(10)
#define ENXX		(1<<8)
#define SSDATA		(1<<9)
#define SSCLK		(1<<10)

#define SSCLK_HIGH()	*gpio_setdataout_addr = (SSCLK)
#define SSCLK_LOW()	*gpio_cleardataout_addr = (SSCLK)
#define ENXX_HIGH()	*gpio_setdataout_addr = (ENXX)
#define ENXX_LOW()	*gpio_cleardataout_addr = (ENXX)
#define SSDATA_HIGH()	*gpio_setdataout_addr = (SSDATA)
#define SSDATA_LOW()	*gpio_cleardataout_addr = (SSDATA)

#define FvcoMax		(5600)
#define Fpd			(10)
#define round(x) (x<0?ceil((x)-0.5):floor((x)+0.5))


class RFMD2080 {
public:
	RFMD2080();
	int setup_spi(void);
	uint16_t read_register(uint16_t address);
	void spi_send_data(uint16_t address, uint16_t data);
	void send_data(uint16_t address, uint16_t data);
	void send_data_with_readback(uint16_t address, uint16_t data);
	void calcRegisters4LO(double f_lo, uint16_t * n_lo, uint16_t * fbkdiv, uint16_t * n, uint16_t * nummsb, uint16_t * numlsb);
	void reset_rfmd2080(void);
	void setup_rfmd2080(double f_lo);
	void newSetup(void);
	void clock_toggle(void);
	void wait_period(void);
	void close_spi(void);
	virtual ~RFMD2080();
private:
	int mem_v;
	volatile void *gpio_addr;
	volatile uint32_t *gpio_setdataout_addr;
	volatile uint32_t *gpio_cleardataout_addr;
	volatile uint32_t *gpio_oe_addr;
	volatile uint32_t *gpio_debounce_addr;
	volatile uint32_t *gpio_datain_addr;
};
#endif /* RFMD2080_H_ */

