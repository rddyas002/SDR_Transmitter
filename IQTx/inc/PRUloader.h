/*
 * PRUloader.h
 *
 *  Created on: 07 Feb 2015
 *      Author: reddi
 */

#ifndef PRULOADER_H_
#define PRULOADER_H_

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>

/* Driver header file */
#include "prussdrv/prussdrv.h"
#include "prussdrv/pruss_intc_mapping.h"

#define LOOP_TIME	(1230e-9)//(0.600e-6)
#define INSTRUCTION_TIME	(5e-9)
#define BLOCK_512K	(524288)
#define PRU_NUM 	0
#define TIMER_OVERHEAD	(30)

#define ALSA_BUFFER_BYTES (2097152)

typedef struct{
    char name[4];
    unsigned int file_size;
    char file_type[4];
} riff_header_s;

typedef struct{
    char format[4];
    unsigned int format_length_rem;
    unsigned short int format_tag;
    unsigned short int channels;
    unsigned int sample_rate;
    unsigned int sample_frame_size;
    unsigned short int block_align;
    unsigned short int bits_per_sample;
} format_section_s;

typedef struct{
    char data[4];
    unsigned int length;
}data_section_s;

typedef enum PRU_MODE{
	PRU_FILE,
	PRU_GEN,
	PRU_STREAM
} PRU_MODE_e;

class PRUloader {
public:
	PRUloader();
	int setup_pruss(void);
	int loadWAVfile2DDR(char * file_name);
	int WAVStream(char * fifoBasePointer, unsigned int * alsaWritePointer, unsigned int capture_rate);
	int genSine2DDR(float frequency, uint32_t samples_per_second, float phase_adj = 0, int suppress_IQ = 0);
	int testOnOff(uint32_t samples_per_second);
	int getDataRemaining(void);
	float blockFree(void);
	double bufferUpdate(void);
	double delta_write(void);
	void stopPRU(void);
	void waitTransferComplete(void);
	void endRxThread(void);
	void destructor(void);
	virtual void pruEvt0_thread(void);
	virtual ~PRUloader();
private:
	FILE * pFile;
	riff_header_s header;
	format_section_s fmt_section;
	data_section_s data_section;

	// external DDR reference
	void *ddrMemOrigin;
	// pru memory reference
	void *pruDataMem;
	uint8_t *pruDataMem_byte;
	uint32_t ddr_mem_size;
	uint32_t phy_add;

	uint32_t data_remaining;
	uint32_t num_of_nops_req;
	uint32_t timer_compare;

	uint32_t arm_write_pointer;
	uint32_t pru_read_pointer;
	struct timespec last_time;
	bool last_block;

	pthread_t irq_thread_evt0;
	bool quit_rx_thread;
	bool rx_thread_running;
	double delta;

	PRU_MODE_e mode;

	// alsa track variables
	char * fifoBasePointer;
	unsigned int * alsaWritePointer;
	unsigned char alsaNextReadPointer;
};

#endif /* PRULOADER_H_ */
