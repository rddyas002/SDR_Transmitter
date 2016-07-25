/*
 * Project: IQTx
 * Description: Capture IQ data from virtual ALSA device and shift through to DAC via PRU
 * Author: Y Reddi
 * E-mail: yreddi@spaceteq.co.za
 */

#include "PRUloader.h"
#include "rfmd2080.h"
#include "AlsaCapture.h"
#include <string.h>

AlsaCapture * alsaCapture = NULL;
PRUloader * pruLoader = NULL;
RFMD2080 * rf_modulator = NULL;

void terminate(void){
	if (alsaCapture != NULL)
		delete alsaCapture;
	if (pruLoader != NULL){
		pruLoader->stopPRU();
		delete pruLoader;
	}
	if (rf_modulator != NULL)
		delete rf_modulator;
}

int main(int argc, char *argv[]) {
	// default capture parameters
	unsigned int rate = 128000;
	unsigned int channels = 2;
	snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE;
	int capt_time;

	// if option is -a [alsa] do USB capture and push through
	if (strcmp(argv[1],"-a") == 0){
		if (argc == 3){
			alsaCapture = new AlsaCapture((char*)"hw:0", &format, rate, channels);

			// wait for buffer to be filled
			while(!alsaCapture->bufferFull());

			// start pushing data through PRU
			// setup PRU
			pruLoader = new PRUloader();
			// setup pruss
			if (!pruLoader->setup_pruss()) {
				terminate();
				return -1;
			}
			// tell pru to feed stream
			if (!pruLoader->WAVStream(alsaCapture->getFIFOBufferAddress(), alsaCapture->getAlsaWritePointer(), alsaCapture->getCaptureRate())){
				terminate();
				printf("Error in WAVStream operation\r\n");
				return -1;
			}

			capt_time = atoi(argv[2]);

			int i = 0;
			for (i = 0; i < capt_time; i++){
				printf("Time: %03d\r\n", i);
				sleep(1);
			}
		}
		else{
			printf("-a: insufficient arguments\r\n");
			terminate();
			return -1;
		}
	}
	else if (strcmp(argv[1],"-s") == 0){	// gen sine wave
		if (argc == 6){
			float freq_float = atof(argv[2]);
			unsigned int samp_uint32 = atoi(argv[3]);
			unsigned int time_limit = atoi(argv[4]);
			float phase_adj = atof(argv[5])*M_PI/180;

			// limit frequency range
			freq_float = (freq_float >= 250000) ? 250000 : freq_float;
			freq_float = (freq_float <= 100) ? 100 : freq_float;
			printf("Sine frequency generating: %.0f\r\n", freq_float);

			// limit sampling frequency
			samp_uint32 = (samp_uint32 >= 1000000) ? 1000000 : samp_uint32;
			samp_uint32 = (samp_uint32 <= 44100) ? 44100 : samp_uint32;
			printf("Sampling frequency: %u\r\n", samp_uint32);

			// limit time
			time_limit = (time_limit >= 300) ? 300 : time_limit;
			time_limit = (time_limit <= 1) ? 1 : time_limit;
			printf("Time limit: %u\r\n", time_limit);

			// setup PRU
			pruLoader = new PRUloader();
			// setup pruss
			if (!pruLoader->setup_pruss()) {
				terminate();
				return -1;
			}
			// tell pru to feed stream
			if (!pruLoader->genSine2DDR(freq_float, samp_uint32, phase_adj)){
				terminate();
				printf("Error in loadWAVFile operation\r\n");
				return -1;
			}

			unsigned int i = 0;
			while (i <= time_limit){
				printf("Time: %03d\r", i++);
				fflush(stdout);
				sleep(1);
			}
			terminate();
			return 0;
		}
		else{
			printf("-s: insufficient arguments\r\n");
			terminate();
			return -1;
		}
	}
	else if (strcmp(argv[1],"-t") == 0){	// test PRU
		if (argc == 4){
			unsigned int samp_uint32 = atoi(argv[2]);
			unsigned int time_limit = atoi(argv[3]);

			// limit sampling frequency
			samp_uint32 = (samp_uint32 >= 1000000) ? 1000000 : samp_uint32;
			samp_uint32 = (samp_uint32 <= 44100) ? 44100 : samp_uint32;
			printf("Sampling frequency: %u\r\n", samp_uint32);

			// limit time
			time_limit = (time_limit >= 300) ? 300 : time_limit;
			time_limit = (time_limit <= 1) ? 1 : time_limit;
			printf("Time limit: %u\r\n", time_limit);

			// setup PRU
			pruLoader = new PRUloader();
			// setup pruss
			if (!pruLoader->setup_pruss()) {
				terminate();
				return -1;
			}
			// tell pru to feed stream
			if (!pruLoader->testOnOff(samp_uint32)){
				terminate();
				printf("Error in testOnOff operation\r\n");
				return -1;
			}

			unsigned int i = 0;
			while (i <= time_limit){
				printf("Time: %03d\r", i++);
				fflush(stdout);
				sleep(1);
			}
			terminate();
			return 0;
		}
		else{
			printf("-t: insufficient arguments\r\n");
			terminate();
			return -1;
		}
	}
	else if (strcmp(argv[1],"-r") == 0){	// control rf modulator
		if (argc == 4){
			rf_modulator = new RFMD2080();
			if (strcmp(argv[2], "1") == 0){
				// Setup rf modulator
				if (!rf_modulator->setup_spi()){
					printf("GPIO memory mapping failed\r\n");
					terminate();
					return -1;
				}

				double freq_lo = atof(argv[3]);
				if ((freq_lo > 1000) || (freq_lo < 100))
					freq_lo = 148.2;

				rf_modulator->reset_rfmd2080();
				rf_modulator->setup_rfmd2080(freq_lo);
				printf("Register 0x%04X contains value 0x%04X\r\n\r\n", RFMD2080_READBACK, rf_modulator->read_register(RFMD2080_READBACK));
			}
			else if (strcmp(argv[2], "0") == 0){
				// Setup rf modulator
				if (!rf_modulator->setup_spi()){
					printf("GPIO memory mapping failed\r\n");
					terminate();
					return -1;
				}
				rf_modulator->reset_rfmd2080();
			}
			else{
				printf("-r ?: unknown argument\r\n");
				terminate();
				return -1;
			}
		}
		else{
			printf("-r insufficient arguments\r\n");
			terminate();
			delete alsaCapture;
		}
	}
	else{
		printf("-?: unknown option\r\n");
		terminate();
		return -1;
	}

	terminate();

	exit(0);
}
