/*
 * PRUloader.cpp
 *
 *  Created on: 07 Feb 2015
 *      Author: reddi
 */

#include "PRUloader.h"
#include <cmath>

extern "C"{
	static tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	// this C function will be used to receive the thread and pass it back to the Thread instance
	void *pruevtout0_thread(void *arg){
		PRUloader* t = static_cast<PRUloader*>(arg);
		t->pruEvt0_thread();
		return 0;
	}
}

PRUloader::PRUloader() {
	pFile = NULL;

	memset((void *)&header, 0, sizeof(riff_header_s));
	memset((void *)&fmt_section, 0, sizeof(format_section_s));
	memset((void *)&data_section, 0, sizeof(data_section_s));

	// external DDR reference
	ddrMemOrigin = NULL;
	pruDataMem = NULL;
	pruDataMem_byte = NULL;

	ddr_mem_size = 0;
	phy_add = 0;
	data_remaining = 0;
	num_of_nops_req = 0;
	timer_compare = 0;

	arm_write_pointer = 0; // next write block
	pru_read_pointer = 0;
	last_block = false;
	mode = PRU_FILE;	// default

	quit_rx_thread = false;
	rx_thread_running = false;
	delta = 0;

	clock_gettime(CLOCK_MONOTONIC_RAW, &last_time);
}

void PRUloader::pruEvt0_thread(){
	// log start time
	struct timespec time_now;
	printf("\r\n");
	clock_gettime(CLOCK_MONOTONIC_RAW, &time_now);
	double start_time = time_now.tv_sec + time_now.tv_nsec / 1e9;
	double dt = 0.0;
	double time_now_s;
	float writeBlock = 0;
	unsigned int writeBlock_u = 0;
	int unwritten_blocks = 0;
	unsigned int blocks_free = 0;

    rx_thread_running = true;
    while(!quit_rx_thread){
    	prussdrv_pru_wait_event(PRU_EVTOUT_0);
    	prussdrv_pru_clear_event(PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);

    	switch (mode){
    		case PRU_FILE:
        		if (last_block){
        			// break out of this loop, data transfer is complete
        			break;
        		}
        		while(blockFree() >= 1){
        			delta = bufferUpdate();
        		}

        		// display time from start
        		clock_gettime(CLOCK_MONOTONIC_RAW, &time_now);
        		time_now_s = time_now.tv_sec + time_now.tv_nsec / 1e9;
        		dt = (time_now_s - start_time);
        		printf("%s%4.0f%s%4.0f\r", "Time = ", dt, ", Interval between writes (ms) = ", delta * 1e3);
        		fflush(stdout);
    			break;

    		case PRU_STREAM:
    			// use the block free signal as an interrupt to check whether sufficient data has been loaded
    			// we here because a block is free - no need to check!
    			blocks_free++;
				// get current write block

    			// determine which block is being written to currently
    			// we end up chasing alsaWritePointer
    			writeBlock = (float)(*alsaWritePointer)/BLOCK_512K;
				writeBlock_u = floor(writeBlock);
				writeBlock_u &= 0x3;

				// is there data that we have not copied to PRU?
				unwritten_blocks = (int)writeBlock_u - (int)alsaNextReadPointer;
				//printf("\r\n%10s >> %5u %5d %5.2f\r\n", "while", blocks_free, unwritten_blocks, writeBlock);
				while (blocks_free && (unwritten_blocks != 0)){
					memcpy((char*)ddrMemOrigin + arm_write_pointer*BLOCK_512K, fifoBasePointer + alsaNextReadPointer*BLOCK_512K, BLOCK_512K);
					blocks_free--;
					alsaNextReadPointer++;
					alsaNextReadPointer &= 0x3;

					// keep track of write pointer
    				arm_write_pointer++;
    				arm_write_pointer &= 0x3;

        			writeBlock = (float)(*alsaWritePointer)/BLOCK_512K;
    				writeBlock_u = floor(writeBlock);
    				writeBlock_u &= 0x3;

					unwritten_blocks = (int)writeBlock_u - (int)alsaNextReadPointer;
				//	printf("%10s >> %5u %5u %5d %5d\r\n", "pru_stream" ,alsaNextReadPointer, arm_write_pointer, unwritten_blocks, writeBlock_u);
				}
    			break;
    		case PRU_GEN:
    			break;
    		default:
    			//printf("PRUloader >> Unknown mode\r\n");
    			break;
    	}
    }
    rx_thread_running = false;
}

int PRUloader::setup_pruss(void) {
	/* Allocate and initialize memory */
	if (prussdrv_init()){
		perror("prussdrv_init failed");
		return -1;
	}
	if (prussdrv_open(PRU_EVTOUT_0)){
		perror("prussdrv_open evt0 failed");
		return -1;
	}
    if (prussdrv_open(PRU_EVTOUT_1)){
        printf("prussdrv_open evt1 failed\n");
        return -1;
    }
	if (prussdrv_pruintc_init(&pruss_intc_initdata)){
		perror("prussdrv_pruintc_init failed");
		return -1;
	}

    /* Start IRQ thread for event handling */
    pthread_attr_t pthread_attr;
    struct sched_param sched_param;
    pthread_attr_init(&pthread_attr);
    int evt_priority = sched_get_priority_max(SCHED_FIFO) - 2;
    pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&pthread_attr, SCHED_FIFO);
    sched_param.sched_priority = evt_priority;
    pthread_attr_setschedparam(&pthread_attr, &sched_param);
    quit_rx_thread = false;
    if(pthread_create(&irq_thread_evt0, &pthread_attr, &pruevtout0_thread, this) != 0){
    	printf("ERR >> Event thread not created\r\n");
    	return -1;
    }
    pthread_attr_destroy(&pthread_attr);

	// map pruss memory
	prussdrv_map_extmem(&ddrMemOrigin);
	// get max size
	ddr_mem_size = prussdrv_extmem_size();
	printf("PRU_DEBUG >> DDR size is 0x%0x bytes\r\nPRU_DEBUG >> DDR virtual address is 0x%0x\r\n", (uint32_t)ddr_mem_size, (uint32_t)ddrMemOrigin);
	// Get physical address
	phy_add = prussdrv_get_phys_addr(ddrMemOrigin);
	printf("PRU_DEBUG >> Start of physical address allocated is 0x%X\r\n",	phy_add);

	memset((char *)ddrMemOrigin, 0, ddr_mem_size);

	return 1;
}

void PRUloader::endRxThread(void){
	if (rx_thread_running){
		 quit_rx_thread = true;
		 // wait for thread to end
		 printf("DEBUG >> waiting for thread to end\r\n");
		 pthread_join(irq_thread_evt0, NULL);
	}
}

int PRUloader::testOnOff(uint32_t samples_per_second){
    uint32_t n = 0;
    uint8_t * next_location = (uint8_t *)ddrMemOrigin;
    mode = PRU_GEN;
    int16_t on = 0x7FFF;
    int16_t off = 0x8000;
    bool I_on = true;

    while(n < ddr_mem_size){
    	if (I_on){
    		memcpy(next_location, &on, 2);
    		next_location += 2;
    		memcpy(next_location, &off, 2);
    		next_location += 2;
    		I_on = false;
    	}
    	else{
    		memcpy(next_location, &off, 2);
    		next_location += 2;
    		memcpy(next_location, &on, 2);
    		next_location += 2;
    		I_on = true;
    	}
    	n += 4;
    }

	// Compute required delay
	double period = 1 / ((double) (samples_per_second));
	timer_compare = (unsigned int) (period/INSTRUCTION_TIME) - TIMER_OVERHEAD;

	// Load data into shared RAM for PRU
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
	pruDataMem_byte = (unsigned char*) pruDataMem;

	pruDataMem_byte[0] = 1;
	pruDataMem_byte[1] = 1;
	// last index written to
	memcpy(&pruDataMem_byte[2], &n, 4);
	memcpy(&pruDataMem_byte[6], &phy_add, 4);
	memcpy(&pruDataMem_byte[10], &timer_compare, 4);

	/* Load and execute binary on PRU */
	int res = prussdrv_exec_program(PRU_NUM, "rf_exciter.bin");
	if (res) {
		perror("prussdrv_exec_program failed");
		return -1;
	}
	// PRU starts pushing out the data from the buffer at this stage
	return 1;
}

int PRUloader::genSine2DDR(float frequency, uint32_t samples_per_second, float phase_adj, int suppress_IQ){
    uint32_t n = 0;
    uint32_t steps = (uint32_t)((float)samples_per_second/frequency);
    uint8_t * next_location = (uint8_t *)ddrMemOrigin;
    mode = PRU_GEN;
    int16_t IQ_data[2] = {0};
    unsigned int bytes_written = 0;
    int supress_I = 1;
    int supress_Q = 1;
    if (suppress_IQ == 1){
    	supress_I = 0;
    }
    if (suppress_IQ == 2){
    	supress_Q = 0;
    }

    for (n = 0; n < steps; n++){
    	// generate I data
    	double I = supress_I*32767*sin(2*M_PI*frequency*(double)n/(samples_per_second-1));
    	IQ_data[0] = (int16_t)floor(I);
    	// generate Q data
    	double Q = supress_Q*32767*sin(2*M_PI*frequency*(double)n/(samples_per_second-1) + phase_adj);
    	IQ_data[1] = (int16_t)floor(Q);
    	memcpy(next_location, &IQ_data[0], 4);
    	next_location += 4;
    	bytes_written += 4;
    }

    printf("PRU_DEBUG >> freq %g\r\n", frequency);
    printf("PRU_DEBUG >> sps %u\r\n", samples_per_second);
    printf("PRU_DEBUG >> steps %u\r\n", steps);
    printf("PRU_DEBUG >> index %u\r\n", bytes_written);

	// Compute required delay
	double period = 1 / ((double) (samples_per_second));
	timer_compare = (unsigned int) (period/INSTRUCTION_TIME) - TIMER_OVERHEAD;
	printf("PRU_DEBUG >> Timer compare value %d\r\n", timer_compare);

	// Load data into shared RAM for PRU
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
	pruDataMem_byte = (unsigned char*) pruDataMem;

	pruDataMem_byte[0] = 1;
	pruDataMem_byte[1] = 1;
	// last index written to
	memcpy(&pruDataMem_byte[2], &bytes_written, 4);
	memcpy(&pruDataMem_byte[6], &phy_add, 4);
	memcpy(&pruDataMem_byte[10], &timer_compare, 4);

	/* Load and execute binary on PRU */
	int res = prussdrv_exec_program(PRU_NUM, "rf_exciter.bin");
	if (res) {
		perror("prussdrv_exec_program failed");
		return -1;
	}
	// PRU starts pushing out the data from the buffer at this stage
	return 1;
}

int PRUloader::loadWAVfile2DDR(char * file_name){
	// used to indicate that interrupt is expected and must be handled
	mode = PRU_FILE;
	// open wav file with read permission
	pFile = fopen(file_name, "r");
	if (pFile == NULL) {
		printf("PRUloader ERROR >> File error\r\n");
	}
	// extract WAV details from header
	fread(&header, 1, sizeof(riff_header_s), pFile);
	fread(&fmt_section, 1, sizeof(format_section_s), pFile);
	fread(&data_section, 1, sizeof(data_section_s), pFile);
	printf("PRU_DEBUG >> Sampling frequency %uHz\r\n",	fmt_section.sample_rate);
	printf("PRU_DEBUG >> Total number of bytes %u\r\n", data_section.length);

	// Compute required delay
	double period = 1 / ((double) (fmt_section.sample_rate));
	timer_compare = (unsigned int) (period/INSTRUCTION_TIME) - TIMER_OVERHEAD;
	printf("PRU_DEBUG >> Timer compare value %d\r\n", timer_compare);

	// number of bytes that need to be written
	unsigned int bytes2write = data_section.length;
	// fill up buffer
	if (bytes2write > ddr_mem_size)
		bytes2write = ddr_mem_size;
	// keep track of data bytes remaining
	data_remaining = data_section.length - bytes2write;
	// read data from file to pruss DDR location
	int samples_written = fread((uint32_t *)ddrMemOrigin, 4, bytes2write / 4, pFile);
	// bytes_written holds the end of the buffer
	unsigned int bytes_written = samples_written * 4;

	// Load data into shared RAM for PRU
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
	pruDataMem_byte = (unsigned char*) pruDataMem;

	pruDataMem_byte[0] = 1;
	pruDataMem_byte[1] = 1;
	// last index written to
	memcpy(&pruDataMem_byte[2], &bytes_written, 4);
	memcpy(&pruDataMem_byte[6], &phy_add, 4);
	memcpy(&pruDataMem_byte[10], &timer_compare, 4);

	/* Load and execute binary on PRU */
	int res = prussdrv_exec_program(PRU_NUM, "rf_exciter.bin");
	if (res) {
		perror("prussdrv_exec_program failed");
		return -1;
	}
	// PRU starts pushing out the data from the buffer at this stage
	// relies on the interrupt to fill buffer
	return 1;
}

int PRUloader::WAVStream(char * fifoBasePointer_p, unsigned int * alsaWritePointer_p, unsigned int capture_rate){
	fifoBasePointer = fifoBasePointer_p;
	alsaWritePointer = alsaWritePointer_p;

	// configure stream mode
	mode = PRU_STREAM;

	// Compute required delay
	double period = 1 / ((double) (capture_rate));
	timer_compare = (unsigned int) (period/INSTRUCTION_TIME) - TIMER_OVERHEAD;
	printf("PRU_DEBUG >> (Stream) Timer compare value %d\r\n", timer_compare);

	// determine which block is being operated on currently
	float write_point = (float)(*alsaWritePointer)/BLOCK_512K;	// between 0-4
	// find start of block
	alsaNextReadPointer = floor(write_point);	// take start of block
	alsaNextReadPointer++;
	alsaNextReadPointer &= 0x3;

	// copy following three blocks from FIFO to PRU
	int i = 0;
	for (i = 0; i < 3; i++){
		memcpy((char *)ddrMemOrigin + i*BLOCK_512K, fifoBasePointer + alsaNextReadPointer*BLOCK_512K, BLOCK_512K);
		// the oldest block in memory is a block forward
		alsaNextReadPointer++;
		alsaNextReadPointer &= 0x3;
	}
	// next block to write to is the 4th block in PRU DDR
	arm_write_pointer = 3;

	// alsaNextReadPointer now holds pointer to the next read location

	// Load data into shared RAM for PRU
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &pruDataMem);
	pruDataMem_byte = (unsigned char*) pruDataMem;

	pruDataMem_byte[0] = 1;
	pruDataMem_byte[1] = 1;
	// last index written to
	unsigned int bytes_written = ddr_mem_size;
	memcpy(&pruDataMem_byte[2], &bytes_written, 4);
	memcpy(&pruDataMem_byte[6], &phy_add, 4);
	memcpy(&pruDataMem_byte[10], &timer_compare, 4);

	/* Load and execute binary on PRU */
	int res = prussdrv_exec_program(PRU_NUM, "rf_exciter.bin");
	if (res) {
		perror("prussdrv_exec_program failed");
		return -1;
	}

	return 1;
}

int PRUloader::getDataRemaining(void){
	return data_remaining;
}

void PRUloader::waitTransferComplete(void){
	while((rx_thread_running || !last_block)){
		sleep(1);
	}
	stopPRU();
}

/* return number of blocks free as float */
float PRUloader::blockFree(void){
	memcpy(&pru_read_pointer, &pruDataMem_byte[14], 4);
	return ((float)pru_read_pointer - (float)arm_write_pointer * BLOCK_512K)/BLOCK_512K;
}

double PRUloader::bufferUpdate(void){
	int samples_written = 0;
	// at 500kHz sampling frequency, will use up 500k*2*2=2Mb/s
	// this implies in 100ms 200kb of 2Mb will be used (just under 10% of buffer used)
	// if data remaining is equal to or greater than a block size then fill whole block else fill remainder

	if (data_remaining >= BLOCK_512K)
		samples_written = fread(((uint8_t *)ddrMemOrigin + arm_write_pointer * BLOCK_512K), 4, BLOCK_512K / 4, pFile);
	else
		samples_written = fread(((uint8_t *)ddrMemOrigin + arm_write_pointer * BLOCK_512K), 4, data_remaining / 4, pFile);

	data_remaining -= samples_written * 4;

	// if data remaining is less than a block then discard (for now)
	if (data_remaining <= BLOCK_512K){
		last_block = true;
	}

	// keep track of write pointer
	arm_write_pointer++;
	arm_write_pointer &= 0x3;

	return delta_write();
}

double PRUloader::delta_write(void) {
	struct timespec time_now;
	clock_gettime(CLOCK_MONOTONIC_RAW, &time_now);
	double delta_time = time_now.tv_sec - last_time.tv_sec + (time_now.tv_nsec - last_time.tv_nsec) / 1e9;
	last_time = time_now;
	return delta_time;
}

void PRUloader::stopPRU(void){
	pruDataMem_byte[0] = 0;
	printf("\r\nWaiting for PRU to HALT\r\n");
	/* Wait for event completion from PRU */
	prussdrv_pru_wait_event(PRU_EVTOUT_1); // This assumes the PRU generates an interrupt connected to event out 0 immediately before halting
	prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
	/* Disable PRU and close memory mappings */
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit();
}

void PRUloader::destructor(void) {
	if (pFile != NULL){
		fclose(pFile);
		pFile = NULL;
	}
}

PRUloader::~PRUloader() {
	destructor();
}

