/*
 * AlsaCapture.cpp
 *
 *  Created on: 02 Mar 2015
 *      Author: reddi
 */

#include "AlsaCapture.h"

extern "C"{
	// this C function will be used to receive the thread and pass it back to the Thread instance
	void *thread_catch_alsa(void *arg){
		AlsaCapture* t = static_cast<AlsaCapture*>(arg);
		t->AlsaReadThread();
		return 0;
	}
}

AlsaCapture::AlsaCapture(char * dev, snd_pcm_format_t * fmt, unsigned int rt, unsigned int ch, int buffer_sizet, int period_sizet, bool log) {
	int err;
	memcpy(&format, fmt, sizeof(snd_pcm_format_t));
	rate = rt;
	channels = ch;
	alsaReadThreadRun = false;
	log_wav = log;
	writeFile = NULL;
	buffer_full = false;

	// allocate 2M ram as buffer
	alsa_fifo_buffer = NULL;
	alsa_fifo_buffer = (char*) malloc(2*1024*1024);
	alsa_write_pointer = 0;
	if (alsa_fifo_buffer == NULL){
		printf("Unable to allocate RAM for FIFO buffer\r\n");
		exit(1);
	}

	if ((err = snd_pcm_open (&capture_handle, dev, SND_PCM_STREAM_CAPTURE, 0)) < 0) {
		fprintf (stderr, "cannot open audio device %s (%s)\n", dev, snd_strerror (err));
	    exit (1);
	}
	DebugPrint("audio interface opened for capture");

	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
	    fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n", snd_strerror (err));
	    exit (1);
	}
	DebugPrint("hw_params allocated");

	if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
	    fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n", snd_strerror (err));
	    exit (1);
	}
	DebugPrint("hw_params initialized");

	if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
	    fprintf (stderr, "cannot set access type (%s)\n", snd_strerror (err));
	    exit (1);
	}
	printf("AlsaCapture: hw_params access set interleaved\r\n");

	if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, format)) < 0) {
	    fprintf (stderr, "cannot set sample format (%s)\n", snd_strerror (err));
	    exit (1);
	}
	DebugPrint("format set");

	if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params, &rate, 0)) < 0) {
	    fprintf (stderr, "cannot set sample rate (%s)\n", snd_strerror (err));
	    exit (1);
	}
	printf("AlsaCapture: rate set to %d bps\r\n", rate);

	if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, channels)) < 0) {
	    fprintf (stderr, "cannot set channel count (%s)\n", snd_strerror (err));
	    exit (1);
	}
	printf("AlsaCapture: channels set to %d\r\n", channels);

	buffer_size = buffer_sizet, period_size = period_sizet;
    err = snd_pcm_hw_params_set_period_size_near (capture_handle, hw_params, &period_size, 0);
    if (err < 0) {
            printf("Unable to set period size\r\n");
    }
    err = snd_pcm_hw_params_set_buffer_size(capture_handle, hw_params, buffer_size);
    if (err < 0) {
            printf("Unable to set buffer size\r\n");
    }

	if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
	    fprintf (stderr, "cannot set parameters (%s)\n", snd_strerror (err));
	    exit (1);
	}
	DebugPrint("hw_params set");

    buffer_size = 0; period_size = 0;
	if (snd_pcm_get_params(capture_handle, &buffer_size, &period_size) == 0){
		printf("Buffer size = %lu Period size = %lu\r\n", buffer_size, period_size);
	}

	snd_pcm_hw_params_free (hw_params);

	if ((err = snd_pcm_prepare (capture_handle)) < 0) {
	    fprintf (stderr, "cannot prepare audio interface for use (%s)\n", snd_strerror (err));
	    exit (1);
	}
	DebugPrint("audio interface prepared");

	if (log_wav){
		// setup write file
		writeFile = fopen("test_file.wav", "wb");
		if (writeFile == NULL){
			ErrorPrint("Error opening write file");
			exit(1);
		}
		// fill wav header information
		sprintf(&header_s.name[0],"RIFF");
		sprintf(&header_s.file_type[0],"WAVE");
		sprintf(&format_s.format[0], "fmt ");
		format_s.format_length_rem = 16;
		format_s.format_tag = 1;
		format_s.channels = 2;
		format_s.sample_rate = rate;
		format_s.sample_frame_size = rate*2*2;
		format_s.block_align = 4;
		format_s.bits_per_sample = 16;
		sprintf(&data_s.data[0], "data");
		// write header information
		total_bytes = 0;
		total_bytes += fwrite(&header_s, 1, sizeof(riff_header_s), writeFile);
		total_bytes += fwrite(&format_s, 1, sizeof(format_section_s), writeFile);
		total_bytes += fwrite(&data_s, 1, sizeof(data_section_s), writeFile);
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

    clock_gettime(CLOCK_MONOTONIC_RAW, &last_time);
	// start read thread
    if (pthread_create(&alsaRead_pthread, &pthread_attr, &thread_catch_alsa, this) != 0){
        ErrorPrint("Receive thread creation error");
    }
}

char * AlsaCapture::getFIFOBufferAddress(void){
	return alsa_fifo_buffer;
}

unsigned int * AlsaCapture::getAlsaWritePointer(void){
	return &alsa_write_pointer;
}

void AlsaCapture::AlsaReadThread(void){
	int err;
	alsaReadThreadRun = true;
	DebugPrint("Receive thread enter");
	int buffer_frames = period_size;

	while (alsaReadThreadRun){
		err = snd_pcm_readi(capture_handle, buffer, buffer_frames);
		// handle overrun
		if (err == -EPIPE){
			fprintf(stderr, "Overrun occurred\n");
			snd_pcm_prepare(capture_handle);
		}
		else if(err < 0){
			fprintf(stderr, "read from audio interface failed (%s)\n", snd_strerror(err));
			exit(1);
		}
		else if(err != buffer_frames){
			fprintf(stderr, "Incomplete read (%s)\n", snd_strerror(err));
		}

		// wrap pointer if overflow
		if (alsa_write_pointer >= 0x200000){
			alsa_write_pointer = 0;	// reset pointer position
			buffer_full = true;		// variable only used once effectively - to tell main that the buffer has been filled once
		}
		// copy alsa data to fifo buffer
		memcpy(alsa_fifo_buffer + alsa_write_pointer, buffer, err*4);
		alsa_write_pointer += err*4;

		if (log_wav){
			total_bytes += fwrite(buffer, 1, err*4, writeFile);
		}
	}

	if (log_wav){
		printf("Total bytes read %d\r\n", total_bytes);
		header_s.file_size = total_bytes;
		data_s.length = total_bytes - 44;
		fseek (writeFile , 0 , SEEK_SET );
		fwrite(&header_s, 1, sizeof(riff_header_s), writeFile);
		fwrite(&format_s, 1, sizeof(format_section_s), writeFile);
		fwrite(&data_s, 1, sizeof(data_section_s), writeFile);
	}
	DebugPrint("Receive thread exit");
}

bool AlsaCapture::bufferFull(void){
	return buffer_full;
}

double AlsaCapture::delta_time(void){
	struct timespec time_now;
	clock_gettime(CLOCK_MONOTONIC_RAW, &time_now);
	double delta_time = (time_now.tv_sec - last_time.tv_sec)*1e6 + (time_now.tv_nsec - last_time.tv_nsec) / 1e3;
	last_time = time_now;
	return delta_time;
}

unsigned int AlsaCapture::getCaptureRate(void){
	return rate;
}


void AlsaCapture::ErrorPrint(const char * str){
	std::cout << "ALSA_ERROR: " << str << std::endl;
}

void AlsaCapture::DebugPrint(const char * str){
	std::cout << "ALSA_DEBUG: " << str << std::endl;
}

void AlsaCapture::stopReceiveThread(void){
	alsaReadThreadRun = false;
    // wait for thread to end
    pthread_join(alsaRead_pthread, NULL);
    DebugPrint("Thread stopped");
}

AlsaCapture::~AlsaCapture() {
	// if thread is still running close it
	if (alsaReadThreadRun){
		stopReceiveThread();
	}
	snd_pcm_close(capture_handle);
	DebugPrint("Alsa closed");
	if (writeFile != NULL){
		fclose(writeFile);
		DebugPrint("writeFile closed");
	}
}

