//
// SDR Cube SW
//
// Juha Niinikoski / OH2NLT
// 03.08.2010
//
// Prosessor dsPIC33FJ256GP710
// Hardware NUE-SDR board set
// Dev tools Mplab v8.40, Microchip C30 v3.20
//


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


// Board specific definitions

#ifndef __SDRCUBE_H
#define __SDRCUBE_H

// processor specific definitions
#include <p33FJ256GP710.h>


// all sort of CPU clock dependent constants
// define FCY, system cycle rate, in the 33F chip Fosc / 2
//#define FCY 29491200UL						// 29,4912 MHz
#define FCY 38707200UL						// 38,7072 MHz

#define DLYCONST ((FCY/1000000UL) + 1)		// constant for timer delay routines
#define TICK_PERIOD (FCY / 1000)			// 1ms tick
#define TONE_CLOCK (FCY / 8)				// tonegenerator clock

// some global feature selections
#define N2APB_BAND_SWITCH					// N2APB style band switching logic
#define MULTI_BAND_RXTX						// P13 bits for Multi Band RX / TX boards

// general constants
#define DEFAULT_VFO_FREQ 14000000			// 20m Soft Rock default

// Min / Max chip output frequencies
#define MIN_SI570_FREQ 3500000				// about 3.5MHz
#define MAX_SI570_180_FREQ 180000000		// Max frequencies for different Si570 types
#define MAX_SI570_280_FREQ 280000000
#define MAX_SI570_810_FREQ 810000000
#define MAX_SI570_945_FREQ 945000000

enum lo_types								// enumerate different lo types
	{
	DDS = 0,
	SI570_180,
	SI570_280,
	SI570_810,
	SI570_945
	};

//#define MAX_SI570_FREQ 1417500000			// about 1,4 Ghz

#define MIN_AD9851_FREQ 0					// no low limit
#define MAX_AD9851_FREQ	72000000			// about 72MHz, 40% of 180MHz DDs clock
//#define MAX_AD9851_FREQ	65000000			// practical max

//#define TUNING_STEPS 4						// number of tuning steps
#define TUNING_STEPS 5						// number of tuning steps, 100Hz step added
#define CW_PITCH	600						// CW pitch
#define CW_SIDETONE 600						// CW sidetone
#define CW_TONE_LOW 90						// addjustment limits, <100 = buzzer off
#define CW_TONE_HIGH 2000

#define CW_PHASE_ADDER 5120					// 625Hz default
#define CW_PHASE_ADDER_LOW 768				// 93,75Hz addjustment limits, <1000 = buzzer off
#define CW_PHASE_ADDER_HIGH 16384			// 2000Hz

#define KEYER 2
#define DEFAULT_BLT 0						// back light timer (s), 0 = off
#define MAX_BLT		120						// max back light timer (s)
#define SERIAL_MODE 0						// serial interface mode
#define BAUD_RATE_STEPS 8					// how maby baud rate selections we have
#define DEFAULT_BAUD_RATE 3					// baud rate list index, default = 9600

#define BATT_MULT 3168L						// battery voltage meter calibration value

#define TX_DISABLE 0x01						// TX disable bit position in band data table

#define PSK_TX_LI_GAIN 23					// PSK TX input gain, 23 = 0dB

// define some tones
// tone set value = FCY / tone(Hz) * 2
#define TONE_600_HZ		(TONE_CLOCK / 600)
#define TONE_800_HZ		(TONE_CLOCK / 800)
#define TONE_1000_HZ	(TONE_CLOCK / 1000)
#define TONE_1200_HZ	(TONE_CLOCK / 1200)
#define TONE_2000_HZ	(TONE_CLOCK / 2000)
#define CW_TONE			(TONE_CLOCK / CW_SIDETONE)

// tone timing
#define DEFAULT_BEEP_LEN 50					// UI beep length(ms)
#define MAX_BEEP_LEN  200					// max UI beep length
#define KLICK 4								// key klick(ms)

// button timing
// timing
#define LONG_PUSH 2000						// button long push timeout (ms)
#define DOUBLE_PUSH 200						// max double push interval
#define CONFIG_PUSH 1000					// encoder switch push time needed to enter Config 
#define DEBOUNCE_DOWN 10					// button down debounce time(ms)
#define DEBOUNCE_UP 50						// button up/release debounce time(ms)

// ADC channels
// front panel potentiometers ( ADC CH# )
#define	FILTER_POT		11					// Filter select pot
#define KEYER_SPD_POT	10					// Keyer speed
#define ATTN_POT		9					// Attenuator addjust pot
#define AF_GAIN_POT		8					// AF gain pot

// battery voltage meter ADC CH
#define BATT			15					// battery voltage

// selector potentiometer hysteresis values
#define VOL_POT_HYST 	15
#define LI_POT_HYST 	30
#define KEYER_POT_HYST 	15
#define FILTER_POT_HYST 15


// Mute delays, 1/sample rate units = 125us
#define RX_MUTE_BASE	500					// mute time when switching back to RX, base value
#define RX_MUTE_VAR		1800				// mute time when switching back to RX, variable value, default setting
#define TUNE_MUTE		100					// mute time after large Si570 tune step (RF signal break)

// AGC test constants
//#define ATTN_START 5000					//  AF agc test, start decrease gain
#define ATTN_START 2000						//  RF agc test, start decrease gain
#define ATTN_SLOPE 256						// decay slope
#define PH_RELEASE	50						// peak hold release rate


// some modules needs these typedefs
typedef unsigned int uint;
typedef unsigned char uchar;


// I/O port init directions & state
#define		TRISA_INIT	0x000C				// I2C2 lines = inputs, others outputs
#define		PORTA_INIT	0x0000

//#define		TRISB_INIT	0x8FFF				// pots, switches & encoder = inputs
#define		TRISB_INIT	0x8F3F				// pots, switches & encoder = inputs, test signals = outputs
#define		PORTB_INIT	0x0000

#define		TRISC_INIT	0x0000
#define		PORTC_INIT	0x0000

#define		TRISD_INIT	0x3300				// LCD control signals & other controls
#define		PORTD_INIT	0x0000

#define		TRISF_INIT	0x0000
#define		PORTF_INIT	0x0000

#define		TRISG_INIT	0x020C				// meter SW & I2C pins = input
#define		PORTG_INIT	0x0000	


// define general I/O signals

// test signals for debugging & performance measurements
#define TEST_PB6	_LATB6					// PGC signal at P6-5
#define TEST_PB7	_LATB7					// PGD signal at P6-4

// system LEDs
#define LD3 _LATG0
#define LD4 _LATG1

//J12 attenuator control
#define ATTN_6DB	_LATC14					// J12-1 6dB relay control, 1 = attenuator on
#define ATTN_10DB	_LATC13					// J12-3 10dB relay control

// Use J13 for test signals Test signals
//#define J13_1	_LATC4						// J13 pins
//#define J13_2	_LATC1
//#define J13_3	_LATC3
//#define J13_4	_LATC2

// P13 band bits, 8-low bits are used, see code
#define J13_BITS LATC

// front panel push button switches
#define	VFO_SW		_RB5					// VFO select button
#define	TUNE_SW		_RB4					// Tune button
#define	MODE_SW		_RB3					// Mode button
#define	ENC_SW		_RB2					// Encoder push button SW
#define	ENC_A		_RB1					// Encoder phase A
#define	ENC_B		_RB0					// Encoder phase B

#define	METER_SW	_RG9					// Meter select

// tone & PWM signals
#define BZ			_LATD2					// Buzzer, I/O not used, signal driven by PWM generator
#define TONE_OUT	_LATD2					// tone I/O

// radio I/O
#define PTT_OUT		_LATD10					// PTT out, SR_PTT signal
#define PSK_PTT		_RD13					// PTT in from the PSK modem
#define MIC_PTT		_RD12					// Microphone PTT

#define AUDIO_SEL	_LATD11					// audio routing select, 1 = SoftRock, 0 = PSK modem


// keyer
//#define DOT		_RD8					// Paddle signals
//#define DASH		_RD9

#define	PADDLE_R	_RD8					// Paddle signals, hardware signals swapped, config option added
#define PADDLE_L	_RD9

// display
#define DISP_BL		_LATF13					// display back light, 1 = on

#define LCD_TRIS	TRISE					// LCD data bus tristate control
#define	LCD_Data	LATE					// LCD data bus

#define	LCD_CS1		LATDbits.LATD7			// LCD control signals
#define	LCD_CS2		LATDbits.LATD6
#define	LCD_RW		LATDbits.LATD5
#define	LCD_DI		LATDbits.LATD4
#define	LCD_E		LATDbits.LATD3


// DDS I/O signals
#define DDS_RESET	_LATA15					// AD9850 DDS Reset
#define DDS_FQ_UD	_LATA14					// AD9850 DDS FQ_UD
#define DDS_W_CLK	_LATA4					// AD9850 DDS W_CLK
#define DDS_SER		_LATA5					// AD9850 DDS D7/SER


//select DDS type
#define AD9851

// DDS ref clock
#define DDS_CLK 180000000UL					// nominal DDS ref clk freq.


// Si570 constants

// Si570 chip address
#define SI570_ADDRESS	(0x55 << 1)			// R/W bit = LSB
#define SI570_CLK	114.285					// Si570 nominal ref osc freq

// codec Bit Bang SPI signals

#define	ML	_LATF12					// CS / latch
#define	MC	_LATF6					// clock
#define	MD	_LATF8					// data

#define	MLDIR _TRISF12				// direction bits
#define	MCDIR _TRISF6
#define	MDDIR _TRISF8


// IQ encoder
#define ENCODER_PORT	PORTB			// IQ rotary encoder inputs
#define ENCODER_BIT_PLACE	0
#define ENCODER_MASK	(3<<ENCODER_BIT_PLACE)	// bit place at port

#define ENCODER_SW	_RB2				// encoder switch


#endif
