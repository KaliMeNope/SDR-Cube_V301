/*
 Cheap DSP dsPICradio board filter routines
 filters are calculated at fir.as module
 on every 8 kHz interrupt.

 Juha Niinikoski OH2NLT 11.09.2005

 IIR filter for Microphone input added 18.09.2005

*/

// Hilbert I/Q filter test added 03.01.2009

// Adapted to SDR Cube project 10.08.2010
// General delay line added 23.08.2010
// change BPF coefficients function added 27.08.2010
// TX I/Q correction multipliers added 15.01.2011
// selectable Mic filter 26.04.2013


//*************************************************************************

// include files
#include "SDR_Cube.h"				// board & processor specific definitions
#include "SDR_Cube_globals.h"		// global data
#include "calc_oh2nlt_filters.h"	// Assembler DSP filter calculation module

// filter coefficients
#include "hilbert69.h"				// 69-tap Hilbert pair
#include "bpf.h"					// 101-tap band pass filter
#include "miciir.h"					// Microphone filter

#include "bpf_200_700.h"			// 500Hz bpf filter
#include "bpf_200_1500.h"			// 1300Hz bpf filter
#include "bpf_200_2400.h"			// 2200Hz bpf filter
#include "bpf_200_3700.h"			// 3500Hz bpf filter


//**************************************************************************
#if 0
// definitions are done in calc_oh2nlt_filters.h file
// left here for documentation purpose

// Note buffer lengths here words but in the assembler module they are bytes
//
//#define BL	256					// buffer  reservation(bytes)
#define BL	128					// buffer  reservation(words), must match with assembler file
#define DLY_LINE 256			// general delay line

// FIR filter data

// Left
extern int firl_taps[BL];		// filter taps
extern int firl_taps_end;		// last members address
extern int firl_n;				// N, Number of taps

extern int firl_dly[BL];		// filter delay line
extern int firl_dly_end;		// last address of ring buffer
extern int firl_dly_ptr;		// ring buffer insert pointer
extern int firl_in;				// left filter input
extern int firl_mul;			// left multiplier
extern int firl_out;			// left filter output
extern int firl_mix;			// left mixer
extern int firl_mix_out;		// left mixer output
extern int x_mul;				// cross multiplier
extern int fir_gain;			// L and R filter gain, range 0 - 32767 ( 0 - 1.0 )

// Right
extern int firr_taps[BL];		// filter taps
extern int firr_taps_end;		// last members address
extern int firr_n;				// N, Number of taps

extern int firr_dly[BL];		// filter delay line
extern int firr_dly_end;		// last address of ring buffer
extern int firr_dly_ptr;		// ring buffer insert pointer
extern int firr_in;				// right filter input
extern int firr_mul;			// right multiplier
extern int firr_out;			// right filter output
extern int firr_mix;			// right mixer
extern int firr_mix_out;		// right mixer output

// Post filter
extern int firp_taps[BL];		// filter taps
extern int firp_taps_end;		// last members address
extern int firp_n;				// N, Number of taps

extern int firp_dly[BL];		// filter delay line
extern int firp_dly_end;		// last address of ring buffer
extern int firp_dly_ptr;		// ring buffer insert pointer
extern int firp_in;				// right filter input
extern int firp_out;			// right filter output


// Delay line
extern unsigned int dly_line[DLY_LINE];		// General delay line
extern unsigned int dly_line_end;			// last address of ring buffer
extern unsigned int dly_line_in_ptr;		// delay line ring buffer insert pointer
extern unsigned int dly_line_out_ptr;		// delay line ring buffer output pointer		
extern int dly_line_in;						// delay line input
extern int dly_line_out_i;					// delay line output


// IIR filter data

// Microphone filter
extern int mic_iir_coef[10];	// filter coefficients, 0,1,2 = A0,A1,A2  3,4 = B1,B2
extern int mic_iir_dly[10];		// filter delay line
extern int mic_iir_in;			// Mic filter input
extern int mic_iir_out;			// Mic filter output

#endif

//****************************************************************************************

// init FIR filters & delay line & correction multipliers

void init_fir(void)
	{
	int x;

//____________________________________________
// Left
	firl_n = N;
	firl_dly_ptr = (int)firl_dly;
	firl_dly_end = firl_dly_ptr + ((2*N)-1);		// end address(last byte) for modulo ring

	firl_taps_end = ((int)firl_taps) + ((2*N)-2);	// end address(last word) for tap vector

// Right
	firr_n = N;
	firr_dly_ptr = (int)firr_dly;
	firr_dly_end = firr_dly_ptr + ((2*N)-1);		// end address(last byte) for modulo ring

	firr_taps_end = ((int)firr_taps) + ((2*N)-2);	// end address(last word) for tap vector

// copy taps
	for(x=0; x<N; x++)
		{
		firl_dly[x] = 0;
		firl_taps[x] = l_taps[x];					// set left ch filter taps

		firr_dly[x] = 0;
		firr_taps[x] = r_taps[x];					// set right ch taps
		}

//____________________________________________
// Post Filter
// Audio PBF

	firp_n = post_N;
	firp_dly_ptr = (int)firp_dly;
	firp_dly_end = firp_dly_ptr + ((2*post_N)-1);		// end address(last byte) for modulo ring

	firp_taps_end = ((int)firp_taps) + ((2*post_N)-2);	// end address(last word) for tap vector

// copy post filter taps
	for(x=0; x<post_N; x++)
		{
		firp_dly[x] = 0;
		firp_taps[x] = bpf_taps[x];						// set band pass filter taps
		}

// set input multipliers
		firl_mul = 0x7FFF;								// default gain = 1.0
		firr_mul = 0x7FFF;
		fir_gain = 0x7FFF;

// software mixers / output multipliers
		firl_mix = 0;									// default gain = 0
		firr_mix = 0;

// RX cross multiplier
		x_mul = 0;										// default L to R cross summation gain is 0

// TX correction multipliers
	txl_mul = 0x7FFF;									// gain correction multiplier, Left, for txbuf0, max gain 1.0
	txr_mul = 0x7FFF;									// gain correction multiplier, Right, for txbuf1, max gain 1.0
	txx_mul = 0;										// phase correction multiplier, Left - Right mixing, no phase correction

//_____________________________________________
// Mic filter
	for(x=0; x<10; x++)
		{
		mic_iir_dly[x] = 0;
		mic_iir_coef[x] = mic_coef_0[x];				// set mic filter coefficients
		}

//_____________________________________________
// General delay line

	dly_line_out_ptr = (unsigned int)dly_line;						// start reading beginning of the buffer
	dly_line_in_ptr = (unsigned int)dly_line+DLY_LINE;				// start inserting samples n ahead (can be changed later)
	dly_line_end = (unsigned int)dly_line + ((2*DLY_LINE)-1);		// end address(last byte) for modulo ring
	dly_line_in = 0;												// start to fill witx zero if no other data available
	}


//_____________________________________________
//
// Set general delay line delay, units = codec sample rate (125us)

void set_audio_delay(unsigned int delay)
	{
	if(delay > DLY_LINE - 1)										// max = dimensioned buffer
		delay = DLY_LINE - 1;

	if(delay == 0)													// delay line data is delay+1 old
		binaural = 0;												// for getting delay completely off we have flag
	else
		binaural = 1;

	__asm__ volatile ("disi #9");									// make pointer uppdate atomic operation
	dly_line_out_ptr = (unsigned int)dly_line;						// reset out pointer, start reading beginning of the buffer
	dly_line_in_ptr = (unsigned int)dly_line + (2*delay);			// calc & set new start inserting samples n ahead (can be changed later)
	}

//_____________________________________________
//
// Cear all FIR filter delay lines (sample buffers, redcus pops & clicks

void clear_all_fir_sample_buffers(void)
	{
	int x;

	for(x=0; x<N; x++)					// phase shift filters
		{
		firl_dly[x] = 0;
		firr_dly[x] = 0;
		}

	for(x=0; x<post_N; x++)				// post filter
		{
		firp_dly[x] = 0;
		}
	}


//_____________________________________________
// Zero filter test/debug 15.11.2005
// write zero coefficients to post filter

void zero_post_fir(void)
	{
	int x;
// zero post filter taps
	for(x=0; x<post_N; x++)
		{
		firp_dly[x] = 0;
		firp_taps[x] = 0;								// set band pass filter taps to ZERO
		}
	}
//
//_____________________________________________
// Change PBF filter coefficients
// simple brute force methode, just copy new
// coefficient set to the filter engine

void set_pbf_coef(int coef_set)
	{
	int x;

	switch(coef_set)
		{
		default:										// prototype test filter, ref
			for(x=0; x<post_N; x++)
				{
				firp_dly[x] = 0;
				firp_taps[x] = bpf_taps[x];
				}
		break;


		case 0:
			for(x=0; x<post_N; x++)
				{
				firp_dly[x] = 0;
				firp_taps[x] = bpf_200_700_taps[x];		//200-700Hz filter (500Hz)
				}
		break;

		case 1:
			for(x=0; x<post_N; x++)
				{
				firp_dly[x] = 0;
				firp_taps[x] = bpf_200_1500_taps[x];	// 200-1500Hz filter (1300Hz)
				}
		break;

		case 2:
			for(x=0; x<post_N; x++)
				{
				firp_dly[x] = 0;
				firp_taps[x] = bpf_200_2400_taps[x];	// 200-2400Hz filter (2200Hz)
				}
		break;

		case 3:
			for(x=0; x<post_N; x++)
				{
				firp_dly[x] = 0;
				firp_taps[x] = bpf_200_3700_taps[x];	// 200_3700Hz filter (3500Hz)	
				}
		break;

		}
	}

//_____________________________________________
// change Mic filter coefficients
//
void set_mic_coef(int coef_set)
	{
	int x;

	switch(coef_set)
		{

		default: case 0:								// direct filter	
			for(x=0; x<10; x++)
				{
				mic_iir_dly[x] = 0;
				mic_iir_coef[x] = mic_coef_0[x];		// set mic filter coefficients
				}
		break;

		case 1:											// filter #1	
			for(x=0; x<10; x++)
				{
				mic_iir_dly[x] = 0;
				mic_iir_coef[x] = mic_coef_1[x];
				}
		break;

		case 2:											// filter #2	
			for(x=0; x<10; x++)
				{
				mic_iir_dly[x] = 0;
				mic_iir_coef[x] = mic_coef_2[x];
				}
		break;

		case 3:											// filter #3	
			for(x=0; x<10; x++)
				{
				mic_iir_dly[x] = 0;
				mic_iir_coef[x] = mic_coef_3[x];
				}
		break;
		}
	}
