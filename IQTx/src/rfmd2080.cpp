
#include "rfmd2080.h"

RFMD2080::RFMD2080() {
	// Initialise pointers
	mem_v = NULL;
	gpio_addr = NULL;
	gpio_setdataout_addr = NULL;
	gpio_cleardataout_addr = NULL;
	gpio_oe_addr = NULL;
	gpio_debounce_addr = NULL;
	gpio_datain_addr = NULL;
}

int RFMD2080::setup_spi(void){
	// map gpio registers
	mem_v = open("/dev/mem", O_RDWR);
	gpio_addr = mmap(0, GPIO0_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem_v, GPIO0_START);
	if (gpio_addr == MAP_FAILED){
		printf("GPIO mapping failed\r\n");
	      	return -1;
    	}
  	gpio_oe_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_OE);
  	gpio_setdataout_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_SETDATAOUT);
  	gpio_cleardataout_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_CLEARDATAOUT);
  	//gpio_debounce_addr = gpio_addr + GPIO_DEBOUNCETIME;
  	gpio_datain_addr = (uint32_t *)((uint8_t *)gpio_addr + GPIO_DATAIN);

	// set all as outputs
	*gpio_oe_addr &= ~(ENXX|SSDATA|SSCLK);

	// set debounce time
	//*gpio_debounce_addr = 10;

	// keep chip select high
	ENXX_HIGH();
	usleep(100000);

	return 1;
}


void RFMD2080::close_spi(void){
	if (mem_v != 0){
		close(mem_v);
		munmap((void *)gpio_addr, GPIO0_SIZE);
	}
}

void RFMD2080::wait_period(void){
	usleep(BIT_TIME);
}

void RFMD2080::clock_toggle(void){
	SSCLK_HIGH();
	wait_period();
	SSCLK_LOW();
	wait_period();
}

uint16_t RFMD2080::read_register(uint16_t address){
	int i = 0;

	// ensure SSDATA as output
	*gpio_oe_addr &= ~(SSDATA);

	// set read flag
	address |= (1<<7);

	// shift address so left aligned
	address <<= 7;	// bit 8 is a dummy and bit 7 is the read/write bit

	// initial clock bit
	clock_toggle();
	ENXX_LOW();
	usleep(5);
	for (i = 0; i < 9; i++){
		if (address & (1<<15))
			SSDATA_HIGH();
		else
			SSDATA_LOW();

		wait_period();
		address <<= 1;	// shift out through MSB
		clock_toggle();
	}
	wait_period();
	SSDATA_LOW();
	clock_toggle();
	// configure SSDATA as an input
	*gpio_oe_addr |= (SSDATA);
	wait_period();
	uint16_t reg_data = 0;
	for (i = 0; i < 16; i++){
		clock_toggle();
		usleep(2);
		// on falling edge data should be available
		if (*gpio_datain_addr & SSDATA){
			reg_data |= 0x0001;
		}
		else{
			reg_data &= 0xFFFE;
		}
		if (i == 15)
			break;
		reg_data <<= 1;		// shift up once
	}
	ENXX_HIGH();
	// ensure SSDATA as output
	*gpio_oe_addr &= ~(SSDATA);
	SSDATA_LOW();
	wait_period();
	clock_toggle();

	return reg_data;
}

void RFMD2080::spi_send_data(uint16_t address, uint16_t data){
	int i = 0;
	// ensure SSDATA as output
	*gpio_oe_addr &= ~(SSDATA);

	// shift address so left aligned
	address <<= 7;	// bit 10 is a dummy and bit 9 is the read/write bit

	// initial clock bit
	clock_toggle();
	usleep(5);
	ENXX_LOW();
	for (i = 0; i < 9; i++){
		if (address & (1<<15)){
			SSDATA_HIGH();
			wait_period();
		}
		else{
			SSDATA_LOW();
			wait_period();
		}
		address <<= 1;	// shift out through MSB
		clock_toggle();
	}
	wait_period();
	wait_period();
	wait_period();
	for (i = 0; i < 16; i++){
		if (data & (1<<15)){
			SSDATA_HIGH();
			wait_period();
		}
		else{
			SSDATA_LOW();
			wait_period();
		}
		data <<= 1;	// shift out through MSB
		clock_toggle();
	}
	ENXX_HIGH();
}

void RFMD2080::send_data(uint16_t address, uint16_t data){
	uint16_t data_read = 0;
	int i = 0;
 	for (i = 0; i < 3; i++){
		spi_send_data(address, data);
		usleep(100);
		data_read = read_register(address);
		if (data_read == data){
			usleep(1000);
			return;
		}
		usleep(100000);
	}
	printf("DEBUG >> Incorrect readback from register 0x%04X after 3 write attemps\r\n", address);
}

void RFMD2080::send_data_with_readback(uint16_t address, uint16_t data){
	uint16_t data_read = 0;
	int i = 0;
 	for (i = 0; i < 3; i++){
		spi_send_data(address, data);
		usleep(100);
		data_read = read_register(address);
		if (data_read == data){
			usleep(1000);
			printf("DEBUG >> Correct readback from register 0x%04X after 3 write attemps | data: 0x%04X\r\n", address, data);
			return;
		}
		usleep(100000);
	}
	printf("DEBUG >> Incorrect readback from register 0x%04X after 3 write attemps\r\n", address);
}

void RFMD2080::reset_rfmd2080(void){
	send_data(RFMD2080_SDI_CTRL, 0x8002);
	usleep(1000);
	send_data(RFMD2080_SDI_CTRL, 0x8000);
	usleep(1000000);
}

void RFMD2080::newSetup(void){
	/* Following RFMD2080 programming guide */
	usleep(100000);		// wait for 100ms
	/* Perform RESET */
	reset_rfmd2080();
	/* (1) Set-up device operation */
	send_data(RFMD2080_P1_FREQ1, 0x3B48);

	/* (2) Set-up additional features */

	/* (3) Set-up operating frequencies */

	/* (4) Set loop filter calibration mode */

	/* (5) Enable device */
}

void RFMD2080::calcRegisters4LO(double f_lo, uint16_t * n_lo, uint16_t * fbkdiv, uint16_t * n, uint16_t * nummsb, uint16_t * numlsb){
	double x = (double)FvcoMax/(2*(f_lo));
	*n_lo = (uint16_t) floor(log(x)/log(2));
	uint16_t lodiv = pow(2,*n_lo);
	double fvco = 2*((double)lodiv)*(f_lo);		//  (MHz)
	if (fvco > 3200)
		*fbkdiv = 4;
	else
		*fbkdiv = 2;
	double n_div = fvco / (*fbkdiv) / Fpd;

	*n = (uint16_t) floor(n_div);
	double tmp1 = pow(2,16) * ((n_div) - (*n));
	*nummsb = (uint16_t) floor(tmp1);
	double tmp2 = pow(2,8) * (pow(2,16) * (n_div - (double)*n) - (double)*nummsb);
	*numlsb = (uint16_t) round(tmp2);
	int n_int = n_div - (double)(((uint32_t)(*nummsb) + ((uint32_t)(*numlsb) >> 8)) >> 16);
	double best_fvco = (double)(*n)*(Fpd)*(*fbkdiv);
	double best_flo = best_fvco/(2*(double)lodiv);
	//printf("Carrier set to %.6fMHz\r\n", best_flo);

	printf("%X,%X,%X,%X,%X\r\n", *n_lo, *fbkdiv, *n, *nummsb, *numlsb);
}

void RFMD2080::setup_rfmd2080(double f_lo){
	uint16_t n_lo;
	uint16_t fbkdiv;
	uint16_t n;
	uint16_t nummsb;
	uint16_t numlsb;
	calcRegisters4LO(f_lo, &n_lo, &fbkdiv, &n, &nummsb, &numlsb);

	// setup device registers
	send_data(RFMD2080_LF, 0xBEFA);
	send_data(RFMD2080_XO, 0x4064);
	send_data(RFMD2080_CAL_TIME, 0x9055);
	send_data(RFMD2080_VCO_CTRL, 0x2D02);
	send_data(RFMD2080_CT_CAL1, 0xB0BF);
	send_data(RFMD2080_CT_CAL2, 0xB0BF);
	send_data(RFMD2080_PLL_CAL1, 0x0028);
	send_data(RFMD2080_PLL_CAL2, 0x0028);
	send_data(RFMD2080_VCO_AUTO, 0xFC00);
	send_data(RFMD2080_PLL_CTRL, 0xD228);
	send_data(RFMD2080_PLL_BIAS, 0x0202);
	/* Program bits for LO */
	uint16_t P1_FREQ1 = (n << 7) | (n_lo << 4) | (0x0008);
	send_data(RFMD2080_P1_FREQ1, P1_FREQ1);
	send_data(RFMD2080_P1_FREQ2, nummsb);
	send_data(RFMD2080_P1_FREQ3, (numlsb << 8));
	send_data(RFMD2080_P2_FREQ1, 0x1ECF);
	send_data(RFMD2080_P2_FREQ2, 0x89D8);
	send_data(RFMD2080_P2_FREQ3, 0x9D00);
	send_data(RFMD2080_FN_CTRL, 0x2A80);
	send_data(RFMD2080_EXT_MOD, 0x0000);
	send_data(RFMD2080_FMOD, 0x0000);
//	spi_send_data(RFMD2080_SDI_CTRL, 0x8000);
	send_data(RFMD2080_GPO, 0x1023);
//	spi_send_data(RFMD2080_T_VCO, 0x0002;
	send_data(RFMD2080_IQMOD1, 0x0283);
	send_data(RFMD2080_IQMOD2, 0xFFCF);
	send_data(RFMD2080_IQMOD3, 0xFFF8);
	send_data(RFMD2080_IQMOD4, 0xFFFF);
	send_data(RFMD2080_DEV_CTRL, 0x1000);
	send_data(RFMD2080_TEST, 0x0005);
	send_data(RFMD2080_SDI_CTRL, 0xC000);
}

RFMD2080::~RFMD2080() {
	close_spi();
	printf("memory map and fd closed\r\n");
}

