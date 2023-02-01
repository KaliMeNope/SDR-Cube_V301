//
//
// 33FJ256GP710 ADC12 routines
// OH2NLT / Juha Niinikoski 18.08.2009
//

// Revision history
// Digital I/O configuration corrected 27.08.2009

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

//Includes
#include "SDR_Cube.h"						// board & processor specific definitions
#include <stdio.h>
#include "uart1.h"							// terminal UART definitions
#include "SDR_Cube_globals.h"				// global data
#include "SDR_Cube_Display_UI.h"			// Display & User interface


static float battery_v = 0;					// scaled battery voltage

// init ADC system for SDR-CUBE board
// remember to set ADC pins as port input
// Reference from 3.3V VDD
// for details see the data sheet


// set ADC clock bug corrected 15.08.2010


void init_adc12(void)
	{
// Note ADC2 bits must also be set if we want any pins to be digital inputs !
	AD2PCFGL = 0x70FF;					// AN8 - AN11, AN15 in use, others digital I/O
	AD1PCFGH = 0xFFFF;					// all digital I/O
	AD1PCFGL = 0x70FF;					// AN0 - AN2 in use, others digital I/O
	AD1CON2 = 0x0000;					// Vref + = VCC,  Vref - = GND
	AD1CON3 = 0x8700;					// RC clock, sample time = 7 TAD
	AD1CON1 = 0x04E0;					// 12-bit mode, integer format, auto convert
	AD1CON1bits.ADON = 1;				// start A/D system
	}


// convert selected channell

int convert_adc12(unsigned int adchs)
	{
	AD1CON1bits.DONE = 0;				// clear done flag
	AD1CHS0 = (adchs & 0x000F);			// select CH, Vref- for MUXA
	AD1CON1bits.SAMP = 1;				// start sampling
	asm("nop");
	while(AD1CON1bits.DONE == 0);		// wait for ready
	return(ADC1BUF0);					// return with buff#0
	}



// Scaling functions

// scale battery voltage
// resolution is 0.1V, 13,0 = 130

#define BATT_SCALE	3168L

int get_scaled_batt(int rbatt)	// 0.1V resolution
	{
	union
		{
		long l;
		unsigned char res[4];
		} batt;

	batt.l = rbatt * BATT_SCALE;
	return (int)(batt.res[2]);							// return with batt/65536
	}


// Test functions
void disp_batt_volt(void)
	{
	int val;
	float fbv;

	val = convert_adc12(BATT);							// convert batt volt ch
	val = get_scaled_batt(val);							// scale
	fbv = (float)val / 10.0;							// for decimal display test

// disp CH15 (Battery voltage) scaled
//	sprintf(pbuf, "Battery voltage = %d (0.1V resolution)\n\r", get_scaled_batt(val));
	sprintf(pbuf, "Battery voltage = %.1fV\n\r", (double)fbv);
	putst1(pbuf);
	}


// Measure, scale, save for others & display battery voltage
// slow floating point scaling

void meas_lcd_disp_batt_volt(void)
	{
	int val;

	val = convert_adc12(BATT);							// convert batt volt ch
	val = get_scaled_batt(val);							// scale
	battery_v = (float)val / 10.0;						// save & for decimal display test
	display_batt_v(battery_v);							// display at the LCD
	}

void disp_all_adc_ch(void)
	{
	int x, val;

	sprintf(pbuf, "\n\rRaw ADC data \n\r");
	putst1(pbuf);

	for(x=8; x<16; x++)
		{
		val = convert_adc12(x);							// convert ch
		sprintf(pbuf, "CH# %d,  Val %d\n\r", x, val);
		putst1(pbuf);
		}

	}
