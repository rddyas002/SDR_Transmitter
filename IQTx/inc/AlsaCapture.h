/*
 * AlsaCapture.h
 *
 *  Created on: 02 Mar 2015
 *      Author: reddi
 */

#ifndef ALSACAPTURE_H_
#define ALSACAPTURE_H_

#include <stdio.h>
#include <stdlib.h>
#include "alsa/asoundlib.h"
#include "PRUloader.h"
#include <pthread.h>
#include <iostream>
#include <cstdio>

class AlsaCapture {
public:
	AlsaCapture(char * dev, snd_pcm_format_t * fmt, unsigned int rt, unsigned int ch, int buffer_sizet = 16384, int period_sizet = 1024, bool log = false);
	void ErrorPrint(const char * str);
	void DebugPrint(const char * str);
	void stopReceiveThread(void);
	double delta_time(void);
	unsigned int getCaptureRate(void);
	char * getFIFOBufferAddress(void);
	bool bufferFull(void);
	unsigned int * getAlsaWritePointer(void);
	virtual void AlsaReadThread(void);
	virtual ~AlsaCapture();
private:
	snd_pcm_t *capture_handle;
	snd_pcm_hw_params_t *hw_params;
	snd_pcm_format_t format;
	unsigned int rate;
	unsigned int channels;

    pthread_t alsaRead_pthread;
    volatile bool alsaReadThreadRun;
    pthread_mutex_t mutex;

    // write file
    FILE* writeFile;
    int total_bytes;
	riff_header_s header_s;
	format_section_s format_s;
	data_section_s data_s;

    // read/write buffer
    char buffer[4096];	// equivalent to 128 frames

    char * alsa_fifo_buffer;	// 2 Mb block (2*1024*1024)
    unsigned int alsa_write_pointer;
    bool log_wav;

    // timing
    struct timespec last_time;
    unsigned long int buffer_size, period_size;
    bool buffer_full;
};

#endif /* ALSACAPTURE_H_ */
