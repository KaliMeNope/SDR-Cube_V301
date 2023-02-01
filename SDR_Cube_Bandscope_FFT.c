//
// SDR_Cube Band Scope FFT calculations
// OH2NLT / Juha Niinikoski 01.09.2010
//
// MPLAB v8.40, Microchip C30 v3.20
//

// Version history
// Microchip sample code bug found, got FFT working 13.09.2010
// OH2UG fast log2 function added 13.09.2010
// fast attach slow release display filter 13.09.2010
// eeprom.defval.bandscope_mode meaning changed to bandscope gain, 0=1, 1=0.5, 2=0.25, 3=0.125 4=0.0625 05.05.2011


/***********************************************************************************************

    "SDR Cube" is embedded DSP-based Radio transceiver for Amateur Radio use.
    This software is meant to be used in the SDR Cube equipment.

    SW designer / Author: Juha Niinikoski, OH2NLT.    Design team: OH2NLT and N2APB
    Copyright (C) 2010 - 2012 by Juha Niinikoski and George Heron, Midnight Design Solutions, LLC.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
        Contacts: 
                Juha Niinikoski, Etuniementie 11C, 02230 Espoo, FINLAND 
                          email: juha.niinikoski@sitecno.fi
                George Heron, 2419 Feather Mae Ct, Forest Hill, MD 21050, USA  
                          email: gheron@midnightdesignsolutions.com
************************************************************************************************/

// include files
#include <dsp.h>
#include <math.h>

#include <stdio.h>
#include "SDR_Cube.h"							// board & processor specific definitions
#include "uart1.h"								// terminal UART definitions
#include "SDR_Cube_LCD.h"						// Graphic LCD driver
#include "SDR_Cube_Display_UI.h"				// Display & User interface
#include "SDR_Cube_globals.h"
#include "SDR_Cube_DCI.h"						// SDR Cube DCI / Codec sample processing module


// FFT definitions
#define FFT_BLOCK_LENGTH	128     			// 128 = Number of frequency points in the FFT
#define LOG2_BLOCK_LENGTH 	7					// 7 = Number of "Butterfly" Stages in FFT processing
#define SAMPLING_RATE		8000				// 8000 = Rate at which input signal was sampled
 

// global data

unsigned char test_scope[128];										// band scope display data

fractional window_mult[FFT_BLOCK_LENGTH];							// pre calculated window function multipliers

fractional sample_i_buf[FFT_BLOCK_LENGTH];							// sample data buffers. For real data only i buffer is used
fractional sample_q_buf[FFT_BLOCK_LENGTH];							// 

fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2] __attribute__ ((section (".xbss, bss, xmemory"), aligned (FFT_BLOCK_LENGTH*2)));		// Declare Twiddle Factor array in X-space


fractcomplex FFT_buffer[FFT_BLOCK_LENGTH] __attribute__ ((section (".ydata, data, ymemory"), aligned (FFT_BLOCK_LENGTH * 2 *2)))={};	// FFT in place calculation buffer

// test variables
int peak_bin;														// max signal bin after FFT
int fft_debug_print = 0;											// debug printouts

// module data


/*   Table of 16 * log2(x) for x from 1.0 to 2.0   */

static const unsigned int log_tbl[] =
	{
	0, 0, 0, 1,
	1, 1, 2, 2,
	2, 3, 3, 3,
	3, 4, 4, 4,
	5, 5, 5, 6,
	6, 6, 6, 7,
	7, 7, 7, 8,
	8, 8, 8, 9,
	9, 9, 9, 10,
	10,10,10,10,
	11,11,11,11,
	12,12,12,12,
	12,13,13,13,
	13,13,14,14,
	14,14,14,15,
	15,15,15,15
	};


//__________________________________ FFT _____________________________________________


// set window multipliers. 0=rectangular, 1=Blackman, 2=Hanning
void init_window_mult(int type)
	{
	int x;

	switch(type)
		{
		case 0:				// Rectangular
			for(x=0; x<256; x++)
				{
				window_mult[x] = 32767;							// set rectangular multipliers
				}
		break;

		case 1:				// Blackman
			BlackmanInit(FFT_BLOCK_LENGTH, window_mult);		// calculate Blackman window multipliers
		break;

		case 2: default:	// Hanning
			HanningInit(FFT_BLOCK_LENGTH, window_mult);			// calculate Hanning window multipliers
		break;
		}
	}


// init for FFT calculations
void init_FFT(void)
	{ 
	HanningInit(FFT_BLOCK_LENGTH, window_mult);					// calculate Hanning window multipliers
//	BlackmanInit(FFT_BLOCK_LENGTH, window_mult);				// calculate Blackman window multipliers
																// Generate TwiddleFactor Coefficients
	TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	// We need to do this only once at start-up
	}


// test
void print_window_mult(void)
	{
	int x;

	sprintf(pbuf, "\n\r___________________________________________________________________");
	putst1(pbuf);

	for(x=0;x<FFT_BLOCK_LENGTH;x++)
		{
		if(x%8 == 0)
			{
			sprintf(pbuf, "\n\r");
			putst1(pbuf);
			}

			sprintf(pbuf, "%6.1d ", window_mult[x]);
			putst1(pbuf);
		}
	}

// test
void print_i_buf(void)
	{
	int x;

	sprintf(pbuf, "\n\r I buf   _________________________________________________________");
	putst1(pbuf);

	for(x=0;x<FFT_BLOCK_LENGTH;x++)
		{
		if(x%8 == 0)
			{
			sprintf(pbuf, "\n\r%3.1d :", x);
			putst1(pbuf);
			}

			sprintf(pbuf, "%6.1d, ", sample_i_buf[x]);
			putst1(pbuf);
		}
	}


// test
void print_q_buf(void)
	{
	int x;

	sprintf(pbuf, "\n\r Q buf   _________________________________________________________");
	putst1(pbuf);

	for(x=0;x<FFT_BLOCK_LENGTH;x++)
		{
		if(x%8 == 0)
			{
			sprintf(pbuf, "\n\r%3.1d :", x);
			putst1(pbuf);
			}

			sprintf(pbuf, "%6.1d, ", sample_q_buf[x]);
			putst1(pbuf);
		}
	}


// test
void print_fft_buf(void)
	{
	int x;

	sprintf(pbuf, "\n\r___________________________________________________________________");
	putst1(pbuf);

	for(x=0;x<FFT_BLOCK_LENGTH;x++)
		{
		if(x%4 == 0)
			{
			sprintf(pbuf, "\n\r");
			putst1(pbuf);
			}

			sprintf(pbuf, "R%6.1d I%6.1d, ", FFT_buffer[x].real, FFT_buffer[x].imag);
			putst1(pbuf);
		}
	}


void print_twiddlefactors(void)
	{
	int x;

	sprintf(pbuf, "\n\rTW_________________________________________________________________");
	putst1(pbuf);

	for(x=0;x<FFT_BLOCK_LENGTH/2;x++)
		{
		if(x%4 == 0)
			{
			sprintf(pbuf, "\n\r");
			putst1(pbuf);
			}

			sprintf(pbuf, "R %.4X I %.4X, ", twiddleFactors[x].real, twiddleFactors[x].imag);
			putst1(pbuf);
		}
	}



// OH2UG  16 * log2(n), 8 bit resolution 

unsigned int intlog2(unsigned int n)
	{
	unsigned int i;

/* prevent endless loop for zero input */
	
	if (n == 0)
		return 0;

/* move most significant one to 1 << 6 and count its position to i   */

	i = 6;

	if ((n & 0xFF80) != 0)
		{
/* ones above target, shift right */
		
		do
			{
			i++;
			n >>= 1;
			}
		while ((n & 0xFF80) != 0);
		}
	else
		{
/* first one maybe below target, shift left */
		
		while ((n & 0x40) == 0)
			{
			i--;
			n <<= 1;
			}
		}

/* combine inetger part from i with fraction from table */

	return (i << 4) + log_tbl[n & 0x3F];
	}



// calculate bandscope data
// exec time about 660us
// 700..750us with fast attach slow release filter

void calc_bandscope(void)
	{
	int x,i,dta;

//	J13_4 = 1;			// test point

// apply window function
	VectorMultiply(FFT_BLOCK_LENGTH, sample_i_buf, sample_i_buf, window_mult);
	VectorMultiply(FFT_BLOCK_LENGTH, sample_q_buf, sample_q_buf, window_mult);

// copy data to FFT calc buffer
	for(x=0; x<FFT_BLOCK_LENGTH; x++)
		{
//		FFT_buffer[x].real =  sample_i_buf[x] >> 1;		// copy windowed real data, multiply with 0.5
		FFT_buffer[x].real =  sample_i_buf[x];			// copy windowed real data, more gain

//		FFT_buffer[x].imag =  sample_q_buf[x] >> 1;		// copy windowed real data, multiply with 0.5
		FFT_buffer[x].imag = sample_q_buf[x];			// copy windowed real data, more gain
		}

// test
	if(fft_debug_print > 1)
		print_fft_buf();

//	Do "in-place" complex FFT on contents of FFT_buffer
	FFTComplexIP (LOG2_BLOCK_LENGTH, FFT_buffer, &twiddleFactors[0], COEFFS_IN_DATA);

//  Do necessary bit reversals, or after FFT
	BitReverseComplex (LOG2_BLOCK_LENGTH, FFT_buffer);

//Compute the square of the magnitude of the complex FFT so we havea Real output vector
	SquareMagnitudeCplx(FFT_BLOCK_LENGTH, FFT_buffer, sample_i_buf);

// test
if(fft_debug_print > 0)
		{
// find peak bin
		VectorMax(FFT_BLOCK_LENGTH, sample_i_buf, &peak_bin);

		sprintf(pbuf, "\n\rPeak bin %d = %d, f = %f Hz", peak_bin, sample_i_buf[peak_bin], ((float)peak_bin * 31.25 ) );
		putst1(pbuf);
		}

// test
	if(fft_debug_print > 1)
		print_i_buf();


// fast scale & copy results

#if 0
// straight copy
	i = 63;
	for(x=0; x<64; x++)
		{
		test_scope[x] =	intlog2(sample_i_buf[i--]) >> 3;
		}

	i = 127;
	for(x=63; x<128; x++)
		{
		test_scope[x] =	intlog2(sample_i_buf[i--]) >> 3;
		}
#endif


// fast attach slow release display
//	i = 63;
	i = 64;										// frequency scale corrected 7.3.2013

	for(x=0; x<64; x++)
		{
		dta = intlog2(sample_i_buf[i--]) >> eeprom.defval.bandscope_mode;	// eeprom variable meaning changed to bandscope gain 05.05.2011
//		dta = intlog2(sample_i_buf[i--]) >> 3;

		if(dta >= test_scope[x])				// uppdate display data
			test_scope[x] = dta;
		else
			{
			if(test_scope[x] > 0)				// if not 0 then decay one step
					test_scope[x]--;
			}	
		}

//	i = 127;
	i = 127;
	test_scope[63] = 0;

//	for(x=63; x<128; x++)
	for(x=64; x<128; x++)						// frequency scale corrected 7.3.2013
		{
		dta = intlog2(sample_i_buf[i--]) >> eeprom.defval.bandscope_mode;	// eeprom variable meaning changed to bandscope gain 05.05.2011
//		dta = intlog2(sample_i_buf[i--]) >> 3;

		if(dta >= test_scope[x])				// uppdate display data
			test_scope[x] = dta;
		else
			{
			if(test_scope[x] > 0)				// if not 0 then decay one step
					test_scope[x]--;
			}	
		}


// test
	if(fft_debug_print > 1)
		{
		for(x=0; x<128; x++)
			{
			sprintf(pbuf, "%d ", test_scope[x]);
			putst1(pbuf);
			}
		}


//	J13_4 = 0;													// test point for timing tests

// display bandscope
	draw_bandscope((unsigned char*)test_scope);
	}



// FFT sampling control

void calc_fft_bandscope(void)
	{
	if(check_sample_buff_full() == 1)							// buffer full
		{
		calc_bandscope();										// calculate FFT & display bandscope
		start_complex_sampling(FFT_BLOCK_LENGTH);				// collect complex samples
		}
	}


void start_fft(void)
	{
	if(check_sample_buff_full() == 0)							// if stopped
		{
		start_complex_sampling(FFT_BLOCK_LENGTH);				// collect complex samples
		}
	}


// FFT tests & FFT test data generator

// generate test data
// freq = frequency, start angle 0.1 units, mode 0=real, 1=complex
void gen_fft_test_data(int freq, int start_angle, int mode)
	{
	double cycle;			// 1/f
	double sr;				// sample rate
	double cyc_sr;			// cycle/sr
	double adder;			// frequency generator adder
	double phase = 0.0;		// wave phase & start value for it
	int x;

	if(start_angle == 0)
		phase = 0.0;							// set start angle 0
	else
		phase = (double)start_angle / 10.0;		// set start angle pi/x

	cycle = 1.0 / (double)freq;				// calculate test frequency cycle
	sr = 1.0 / (double)SAMPLING_RATE;		// calculate sample rate, 125us @ 8000Hz
	cyc_sr = cycle / sr;					// calculate cycle / samplerate constant
	adder = (2.0*PI) / cyc_sr;				// calculate DDS adder

	for(x=0; x<FFT_BLOCK_LENGTH; x++)
		{
		sample_i_buf[x] = (int)(sin(phase) * 32767.0);	// calculate I table
		if(mode == 1)
			sample_q_buf[x] = (int)(cos(phase) * 32767.0);	// calculate q table
		else
			sample_q_buf[x] = 0;			// real data , zero imag table

		phase = phase + adder;				// increment phase accumulator
		}
// test
	print_i_buf();
	print_q_buf();
	}

// select debug printout
void set_fft_debug_printout(int level)
	{
	fft_debug_print = level;
	}
