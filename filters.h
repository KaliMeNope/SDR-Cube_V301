/*
 Cheap DSP dsPICradio board filter routines
 filters are calculated at fir.as module
 on every 8 kHz interrupt.

 Juha Niinikoski OH2NLT 11.09.2005

 IIR filter for Microphone input added 18.09.2005
*/

// Hilbert I/Q filter test added 03.01.2009

// Adapted to SDR Cube project 10.08.2010


#ifndef __FILTERS_H
#define __FILTERS_H

void init_fir(void);						//init FIR filters & delay line
void clear_all_fir_sample_buffers(void);	// Cear all FIR filter delay lines (sample buffers, redcus pops & clicks

void set_audio_delay(unsigned int);			// Set general delay line delay, units = codec sample rate (125us)
void set_pbf_coef(int);						// set audio PBFcoefficients (filter #)
void set_mic_coef(int);						// set Mic filtercoefficients (filter #)
#endif
