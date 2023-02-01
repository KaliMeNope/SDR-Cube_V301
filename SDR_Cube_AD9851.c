//
// AD9850  DDS chip driver
// OH2NLT / Juha Niinikoski 02.04.2004
// AD9851 test code added 26.08.2004


// Revision history
// DDS frequency to N calculation added 14.08.2010


/***********************************************************************************************

    "SDR Cube" is embedded DSP-based Radio transceiver for Amateur Radio use.
    This software is meant to be used in the SDR Cube equipment.

    SW designer / Author: Juha Niinikoski, OH2NLT.    Design team: OH2NLT and N2APB
    Copyright (C) 2010-2012 by Juha Niinikoski and George Heron, Midnight Design Solutions, LLC.

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


#include <math.h>
#include "SDR_Cube.h"							// board & processor specific definitions

#include <stdio.h>
#include "uart1.h"								// terminal UART definitions
#include "SDR_Cube_globals.h"					// global data


// module variables
	struct dds_regs
	{
	long dds_freq;
	unsigned char dds_config;
	};

	union									// AD9850 frequency & config data, index 0 = LSB
	{
	struct dds_regs dds_regs;
	unsigned char ad9850[5];
	} dds;



// AD9850 data transfer

void dds_send_one( unsigned char dta )		// send one byte LSB first
	{
	char x;
	for(x=0; x<8; x++)						// do all bits
		{
		if(dta & 1)
			DDS_SER = 1;
		else
			DDS_SER = 0;

		asm("nop");
		DDS_W_CLK = 1;						// generate clock pulse
		dta = dta >> 1;						// move next bit
		asm("nop");
		DDS_W_CLK = 0;	
		}
	}


// load AD9850 from buffer

void dds_load(unsigned long freq)			
	{
	int x;

	dds.dds_regs.dds_freq = freq;

	for(x=0; x<5; x++)
		{
		dds_send_one(dds.ad9850[x]);		// send 40 bits
		}

	DDS_FQ_UD = 1;							// generate update pulse
	asm("nop");								// very small delay
	asm("nop");
	DDS_FQ_UD = 0;
	}


// Init AD9850 interface

void dds_init(void)
	{
	DDS_RESET = 1;								// reset AD9850
	DDS_SER = 0;
	DDS_RESET = 0;

	DDS_FQ_UD = 1;								// generate update pulse
	DDS_FQ_UD = 0;								// set serial mode

#ifdef AD9851	
	dds.dds_regs.dds_config = 0x01;				// control & phase bits = 0x01, 6xmultiplier enabled
#else
	dds.dds_regs.dds_config = 0;				// control & phase bits = 0
#endif

	}


#define TWO_EXP32 4294967296.0

static double dds_clk;							// DDS ref clock
static float dds_mult;							// pre calculated DDS multiplier (2^32 / clk),  bits / Hz



// DDS frequency calculations
//
// equations
// 
// f = (N*clk) / 2^32
// N = (2^32 / clk) * f
//
// dds_mult = 2^32 / clk
// N = f * dds_mult
//


// set DDS clock frequency & calculate internal multipliers
void set_dds_clk(unsigned long clk)
	{
	dds_clk = clk;								// convert ref clk to float and save it
	dds_mult = (double)TWO_EXP32 / dds_clk;		// calculate with enough accaracy

// debug
//	sprintf(pbuf, "\n\rDDS mult = %f", (double)dds_mult);
//	putst1(pbuf);
	}

// calculate DDS N
unsigned long calc_dds_n(unsigned long freq)
	{
	unsigned long n;
	float f, fn;

	f = freq;									// do number format conversions & calculate N
	fn = dds_mult * f;
	n = fn;

// debug
//	sprintf(pbuf, "\n\rf = %ld, fn = %f, n = %ld",freq, (double)fn, n);
//	putst1(pbuf);
	return n;
	}

// set DDS output frequency, calculate & set N
void set_dds_freq(unsigned long freq)
	{
	dds_load(calc_dds_n(freq));					// calculate & set DDS
	}
