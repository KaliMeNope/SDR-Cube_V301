//
// SDR Cube SW
//
// Juha Niinikoski / OH2NLT
// 10.08.2010
//
// Prosessor dsPIC33FJ256GP710
// Hardware SDR-Cube board set
// Dev tools Mplab v8.40, Microchip C30 v3.20
//
// Project build options
// - Large Data Model
// - use 64-bit Double, not with v1.02 and later
// - heap 500
// - modify linker script for DSP library search
//

// Main SW module

// OH2NLT Revision history
// Linker script modified for Microchim AN1097 Flasher oh2nlt / 11.08.2010
// Rotary encoder logic added 13.08.2010
// Excec speed (FCY) increased to 38,7072 MHz 13.08.2010
// Improved UI tone generator. Use TMR2 and OC3 for hardware tone. 15.08.2010
// Graphical UI & restructuring the project modules 21.08.2010
// UI (Display) format tuning 22.08.2010
// Audio delay line & Binaural RX tests 24.08.2010
// Memory & VFO select logic & user config menu system 25.08.2010
// Addjustable audio PBF, four selectable filter sets 27.08.2010
// Both ends sin(x)/x shaped CW 28.08.2010
// New style user menu system, keyer logic enhanced 29.08.2010
// First RX I/Q correction calculations, binaural off 125us offset corrected 31.08.2010
// Linker script modified to search also dsp library 01.09.2010
// RF step attenuator control (J12 signals) 02.09.2010
// CW sidetone to headphones test 03.09.2010
// Bandscope FFT calculations started to work 13.09.2010
// Si570 Soft tune v1.0 17.09.2010, rewritten 18.09.2010
// FFT window type selection, Si570 XTAL cal routine, Menu cleaning 18.09.2010
// TX voice spectrogram display & some button interlocks 19.09.2010
// Codec Mic amp to Line amp bias mismatch problem patch & display back light logic enhanced 21.09.2010
// RF attenuator + Li gain put to RF attn pot,  22.09.2010
// Si570 / DDS LO select, LO multiplier select 20.10.2010
// User interface format changed, new style tuning indicator 06.11.2010
// Project file names cleaned, test & debug interface cleaned v1.00 / 06.11.2010
// Copyright info added v1.00 / 07.11.2010
// Factory default reset added, menu exit data save condition added, some text format changes v1.00 / 12.11.2010
// Small text changes, button debounce logic enhanced v1.00 14.11.2010
//
// Si570 Ref Osc limits changed +/-2000 ppm 114.000 to 114.515 14.01.2011
// AF Gain pot scaling changed. ADC/51+47 remove empty offset. 14.01.2011
// LCD character generator file format corrected & CHR gen commented 14.01.2011
// LD3 = overload indicator for both ADC channels 14.01.2011
// LD3, LD4 used for start up failure indication, LD4 = SW heart beat 14.01.2011
// Band scope added to the Codec pass through test 14.01.2011
// Paddle signals reverse option, straight key buzzer sound  14.01.2011
// TX IQ correction multipliers added, major modifications around the code 15.01.2011
// User Menu text changes, debug terminal cleaning 27.01.2011
// Reverse CW (CWR mode) added 27.01.2011 / v1.01
// -----------------------------SW v1.01 release OH2NLT / 27.01.2011----------------------------

// Band select logic & band select table editor, VFO tuning limits added  31.01.2011 / v1.02 test version
// Band voltage output added. Yaesu compatible band select voltage is present at P11 DAC1_B pin 06.02.2011
// US format date 06.02.2011 => 06-FEB-2011
// 100MHz digit added to the frequency display, tuning limit calculations enhanced, some display corrections 09.02.2011
// Added Si570 calculations n1 check <= 128 M. Collins KF4BQ suggested fix 10.02.2011
// double variablec changed to long double. 64bit double removed from project options. 13.02.2011
// TX disable bit added to the band data table 13.02.2011
// N2APB style band switch logic coded 15.02.2011
// N2APB style band tables extended, separate tuning limits for Si570 & AD9851 DDS 16.02.2011
// Si570 and IC1 EEPROM I2C bus timeout error handling added 17.02.2011
// Si570 type selection, menu & message format clean up 19.02.2011
// N2APB SDR Cube User Parameters printout added,  Very fast tuning stet added 21.02.2011
// On the fly EEPROM factory default reset added, N2APB factory frequency setup scheme 22.02.2011
// PSK TX "RF Attn volume control" bug corrected, some front panel TX locks added 27.02.2011
// -----------------------------SW v1.02 release OH2NLT / 27.02.2011----------------------------

// intermediate fix for TR -> RX transition codec DC bias match 14.03.2011
// PTT functions active while inside Menu adjustments 14.03.2011
// -----------------------------SW v1.03 release OH2NLT / 14.03.2011----------------------------

// corrected bug in PSK TX audio level, Codec Mag set value was wrongly used when PSK TX 21.03.2011
// corrected bug in PSK TX opposite sideband supression, RX FIR need to be set to unity gain 21.03.2011
// TX->RX delay printout added to the info command 02.04.2011 
// -----------------------------SW v1.04 release OH2NLT / 02.04.2011----------------------------

// soft DDS audio quality tests 03.05.2011
// CW pitch set value changed to phase adder value, clean CW tone scheme 05.05.2011 
// eeprom.defval.bandscope_mode meaning changed to bandscope gain, 0=1, 1=0.5, 2=0.25, 3=0.125 4=0.0625 05.05.2011
// -----------------------------SW v1.05 release OH2NLT / 05.05.2011----------------------------

// TX Lock menu selection added, can select if TX is allowed ham bands only or no limits 9.6.2011
// -----------------------------SW v1.06 release OH2NLT / 09.06.2011----------------------------

// Yaesu style CW 12.09.2011
// -----------------------------SW v1.07 release OH2NLT / 09.06.2011----------------------------

// Underscore filter display 22.12.2011
// S / dBm meter 17.01.2012
// keyer max speed dropped to 63wpm. Easier to set with the potentiometer 22.01.2012
// 100Hz tuning step added 18.02.2012
// -----------------------------SW v1.08 release OH2NLT / 18.02.2012----------------------------

// 1.09 test version 

// Tune RF power adjustment added 23.11.2012
// EEPROM fast page write added 23.11.2012
// filters.s module CORCON ACCSAT mode corrected 24.12.2012
// Codec magnitude calculations transfered to optimized assembler module codec_mag.s 25.11.2012
// Automatic RF level control tests 30.11.2012 
// AGC action Bar meter added 1.12.2012
// AGC adjustments added to the user menu 01.12.2012
// soft DDS phase sync added to DCI_CW_ON case 02.12.2012 
// AGC TX -> RX switching logic added 02.12.2012
// AGC attack defeat timer logic added 05.12.2012
// S-meter blanked when TX 05.12.2012
// AGC parameters added to User Parameter printout 06.12.2012

// version changed to 2.00
// -----------------------------SW v2.00 release OH2NLT / 06.12.2012----------------------------

// RX IQ balance AGC cauced adjustment bug corrected 26.12.2012
// AGC default factory setting changed to On (1) 27.12.2012
// RX & TX IQ multiplier set limits changed 27.12.2012

// -----------------------------SW v2.01 release OH2NLT / 27.12.2012----------------------------

// EEPROM I2C driver timeout corrected for the EEPROM page write 15.01.2013
// small bug in the filter potentiometer code corrected 20.01.2013

// -----------------------------SW v2.02 release OH2NLT / 20.01.2013----------------------------

// AM modulator tests 03.03.2013
// AM mode added 06.03.2013
// Spectrum display frequency accuracy enhanced & CW carrier marker added 07.03.2013
// AM TX RF gain added 09.03.2013 
// AM RX sidebands corrected 10.03.2013

// -----------------------------SW v2.03 release OH2NLT / 06.03.2013----------------------------

// version number changed to 3.00, no other changes 26.03.3013

// -----------------------------SW v3.00 release OH2NLT / 26.03.2013----------------------------

// Mic filter code & some bug fixes 20.04.2013
// selectable Mic pre-emphasis filter 26.04.2013
// Multi Band RX & TX board filter control bits (DSP board P13) 01.07.2013
// release set of the Mic filters 31.07.2013

// -----------------------------SW v3.01 release OH2NLT / 31.07.2013----------------------------

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
#include <stdio.h>
#include <math.h>
#include "SDR_Cube.h"							// board & processor specific definitions
#include "SDR_Cube_main_functions.h"			// main module callable functions
#include "uart1.h"								// terminal UART definitions
#include "uart2.h"								// PSK UART definitions
#include "tmr5delay.h"							// simple delay functions with TMR5
#include "SDR_Cube_adc12.h"						// ADS system & scaling functions
#include "SDR_Cube_debug_terminal.h"			// test & debug terminal I/O
#include "SDR_Cube_AD9851.h"					// AD9850 / AD9851 DDS driver
#include "SDR_Cube_I2C_2.h"						// Si570 I2C driver
#include "SDR_Cube_Si570.h"						// Si570 frequency calculations & setup
#include "SDR_Cube_LCD.h"						// Graphic LCD driver
#include "SDR_Cube_Display_UI.h"				// Display & User interface
#include "SDR_Cube_user_menu.h"					// User LCD menu system
#include "SDR_Cube_timers.h"					// Timer & pwm system
#include "filters.h"							// DSP filters
#include "tlv320aic23b.h"						// Codec driver & audio switching
#include "tlv320aic23b_registers.h"				// Codec chip register definitions
#include "SDR_Cube_DCI.h"						// SDR Cube DCI / Codec sample processing module
#include "calc_oh2nlt_filters.h"				// Assembler DSP filter calculation module
#include "SDR_Cube_I2C_1.h"						// 24LC256 EEPROM I2C driver
#include "SDR_Cube_EEPROM.h"					// EEPROM functions
#include "SDR_Cube_eeprom_def.h"				// EEPROM data definitions
#include "SDR_Cube_Bandscope_FFT.h"				// Bandscope FFT functions
#include "SDR_Cube_IO.h"						// Geberal I/O functions


// processor config
	_FBS		( RBS_NO_RAM & BSS_NO_FLASH & BWRP_WRPROTECT_OFF )
	_FSS		( RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF )
	_FGS		( GSS_OFF & GWRP_OFF )
	_FOSCSEL	( FNOSC_PRIPLL & IESO_OFF )
//	_FOSCSEL	( FNOSC_PRIPLL & IESO_OFF & TEMP_OFF )
	_FOSC		( FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT )
	_FWDT		( FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768 )
	_FPOR		( FPWRT_PWR32 )
//	_FPOR		( PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR32 )

// User ID can not be used with the Flasher
//	_FUID0		( 0x03 )
//	_FUID1		( 0x08 )
//	_FUID2		( 0x20 )
//	_FUID3		( 0x10 )



// version info
const char ver[] =    {"v3.01   "};		// software version information, displayed at startup
//const char date[] = {"07.02.2011"};		// version date, EU format
const char date[] = {"31-JUL-2013"};		// version date, US format


// global variables

// test
char qam_buff[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15, 3,6,7,12, 15,0,2,9, 4,8,10,12, 5,1,0,14}; //test data

// print buffer
char pbuf[100];								// global buffer for sprintf printouts

// timer system
unsigned int ms_counter;					// general mod 65536 ms counter
unsigned int seconds;						// free running second counter

unsigned int long_push;						// long push timer, count from set value to zero, global visibility
unsigned int double_push;					// double push timer, count from set value to zero, global visibility

int cw_tone = CW_TONE;						// CW sidetone

int bl_timer;								// LCD back light timer
int bl_off_flag;							// turn backlight off flag


// Codec & audio switching
int dci_mode = DCI_SSB_RX;					// audio switch, default = SSB RX
int binaural = 0;							// Binaural RX feature, 0 = off, 1 = on


// soft DDS oscillator
double phase;								// for soft DDS init calculations
int dds_table[256];							// dds waweform
unsigned int 	ph_acc;						// phase accumulator
unsigned int 	ph_adder;					// phase adder
int	dds_i;
int	dds_q;									// soft audio dds outputs


// volume control
unsigned int hp_vol;						// headphone volume #0 - 127 = -73 to 6 dB
unsigned int li_vol;						// line input volume #0 - 31  = -34,5 to 12 dB

// RF attenuator bits
int rf_attn_bits = 0;						// rf attenuator bits

// codec control registers copy
unsigned int 	tlv_llicvc;
unsigned int	tlv_rlicvc;
unsigned int	tlv_lchvc;
unsigned int	tlv_rchvc;
unsigned int	tlv_aapc;
unsigned int	tlv_dapc;
unsigned int	tlv_pdc;
unsigned int	tlv_daif;
unsigned int	tlv_src;
unsigned int	tlv_dia;


// status flags
unsigned char mic_old_ptt = 1;				// history value of Mic PTT signal
unsigned char mic_ptt =1;					// atomic sample Mic PTT signal

unsigned char psk_old_ptt = 1;				// history value of PSK PTT signal
unsigned char psk_ptt =1;					// atomic sample PSK PTT signal

unsigned char tx_disable = 0;				// TX disable flag, 1 = disabled.

// operating mode etc

int op_mode = 0;							// operating mode for sw logic , 0 = LSB, 1 = USB, 2 = CW, 3 = CWR
											// base value from the EEPROM struct: eeprom.defval.mode[eeprom.defval.mem]
int keyer_tx_req = 0;						// keyer TX request
int keyer_tx_on = 0;						// keyer TX granted / ON
int rx_tx = 0;								// display selector, 0=RX, 1=TX, 2=PSK_TX, 3=TX_CW, 4=Tune, 9=Error/TX disabled
int vfo_lock = 0;							// VFO lock flag
unsigned int wpm = 20;						// keyer speed
int shape_ptr = 0;							// keyer sin(x)/x shape table pointer
int rit_on = 0;								// RIT on / off
int si570_init_done = 0;					// flag neded for possible runtime LO change


// VFO limits
long min_vfo_freq = 0;						// VFO limits, display frequency, actual limit depend upon LO type & multiplier
long max_vfo_freq = 999999999;

// EEPROM data structures
union
	{
	struct defval defval;
	unsigned char storage[sizeof(struct defval)];
	}eeprom;

// system calibration values
union
	{
	struct calval calval;
	unsigned char ee[sizeof(struct calval)];
	}cal;


static unsigned int d_cs;					// eeprom checksums
static unsigned int c_cs;


// some system wide constant strings
//const long vfo_tuning_steps[TUNING_STEPS] = {10, 1000, 100000, 1000000};						// VFO tuning steps
const long vfo_tuning_steps[TUNING_STEPS] = {10, 100, 1000, 100000, 1000000};					// VFO tuning steps, 100Hz step added
const char keyer_types[4][8] ={"Dot pri\0", " Iamb A\0", " Iamb B\0", " Manual\0"};				// keyer modes
const long baud_rates[BAUD_RATE_STEPS] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};	// standard UART baudrate steps


// bandscope
unsigned char b_scope_data[128];			// test bandscope data
unsigned char band_scope_data[128];			// band scope data
int band_scope_data_idx = 0;				// input index to scope data array
int draw_dummy_flag = 0;					// dummy scope uppdate flag

// potentiometers
static int old_vol_pot = -99;				// old volume pot value
static int old_li_pot = -99;				// old line in pot value
static int old_k_pot = -99;					// old volume pot value
static int old_f_pot = -99;					// old volume pot value, must change with first measurement
static int old_filter = 9;					// old filter value, force set in the first round

// test & other
long test_mul = 0;
int f_disp_type = 0;						// select display type, for testing
int yaesu_type_cw = 1;						// Yaesu type CW logic, 1= Yaesu, 0 = Cube CW
int yaesu_cw_offset = 0;					// Yaesu type CW LO offset

// Power / S-meter meter
#define S9_REF	58.9						// S9 ref level, for filtered input (in band)
#define S9_dBm 73.0							// S9 display dBm level

unsigned long iq_energy_2;
float rf_attn_cur_val;						// RF attenuator relays + Codec Li amp current set calue (dB)
double raw_db;								// input calculated to dB scale
int s_units;								// 1 - 9 (S1-S9)
int plus_units;								// +10 ... +60
int agc_units;								// dB scaled value from calculations
int fir_gain_cor = 0;						// fir multiplier gain correction for the S-meter display

// AGC
unsigned int agc_set_val;
int agc_attack_timer = 0;					// attack delay timer, units = agc samples
unsigned long agc_input_val;
unsigned int agc_peak_val = 0;				// last peak value
unsigned int agc_attack_val = 0;			// last attack phase input value
int agc_action_indicator = 0;				// display indicator for current AGC action, value = 0 ... 29
unsigned int agc_save_value;				// AGC value stored here when going to TX, AGC set value when coming back to RX
int agc_on = 1;								// enable agc calculations

// magnitude selector variables, defined in codec_mag.s
extern int mag_sel;						// Magnitude select. 0= direct, 1=mul by2, 2=mul by 4 etc
extern unsigned int codec_in_peak;		// rectified input peak value before scaling

// helper functions ______________________________________________________________________

// set AGC gain & calculate S-meter correction value
// input = gain 0 ... 65535

void set_agc_gain(unsigned int gain)
	{
	int mag;
	int mult;
	int m;
	int g;

// divide overal gain to codec mag and filter_gain components

//	mag = gain / 16384;				// solve with division
//	m = gain % 16384;
//	mult = 16384 + m;

	m = gain & 0x3FFF;				// alternative method without division
	mag = gain >> 14;
	mult = 16384 + m;	


// set Codec mag & filter gain
	__asm__ volatile("disi #5");	// set mag & gain as atomic operation
	mag_sel = mag;					// set directly to codec_mag.s variable
	fir_gain = mult;				// set fir gain

// calculate correction factor for the S-meter
	m = mult;

	if((m - 29204) > 0)
		g = 0;										// 0 dB
	else
		{
		if((m - 26028) > 0)
			g = 1;									// 1 dB
		else
			{
			if((m - 23197) > 0)
				g = 2;								// 2 dB
			else
				{
				if((m - 20675) > 0)
					g = 3;							// 3 dB
				else
					{
					if((m - 18426) > 0)
						g = 4;						// 4 dB
					else
						{
						if((m - 16422) > 0)
							g = 5;					// 5 dB
						else
							{
							if((m - 14636) > 0)
								g = 6;				// 6 dB
							else
								g = 7;				// 7dB or more
							}
						}
					}
				}
			}
		}

	fir_gain_cor = g;								// SET s-METER CORRECTION VALUE

	}


// save AGC state when switching to TX
void set_agc_for_tx(void)
	{
	agc_on = 0;							// agc active flag off
	agc_save_value = agc_set_val; 		// save current agc set value			
	set_agc_gain(32767);				// set Codec Mag to 0, filter gain to max 32767
	display_bar_meter(0);				// no AGC indicator
	}

// restore AGC state / restart AGC calculations when returning to RX
void set_agc_for_rx(void)
	{
	if(eeprom.defval.agc_state != 0)					// only if AGC is on
		{
		set_agc_gain(agc_save_value);					// use value what we had when left from RX
		agc_sample_rdy = 0;								// clear sample ready flag
		agc_samples = eeprom.defval.agc_sample_time;	// reload sample counter
		max_agc_sample = 0;								// reset max sample value
		agc_attack_timer = 0;							// reset timers
		agc_hang_timer = 0;
//		agc_on = 1;						// enable agc calculations, this is done when the mute_ctr expire
		}
	}


// N2APB style band switch tables / 16.02.2011

#define N2APB_TABLE_SIZE 15

// Band upper limit table
long n2apb_bands[N2APB_TABLE_SIZE] = {1790000, 3490000, 6990000, 9990000, 13990000, 18058000, 20990000, 24880000, 27990000, 49990000, 139999000, 221990000, 419990000, 901990000, 999999999};

// TX disable table
// TX is disabled above following values in segments defined above
long n2apb_txinh[N2APB_TABLE_SIZE] =       {0, 2010000, 4010000, 7310000, 10160000, 14360000, 18178000, 21460000, 25000000, 29800000, 54010000, 148010000, 225010000, 450010000, 928010000};

// PWM dac values for band voltage
unsigned int n2apb_dac_val[N2APB_TABLE_SIZE] = {0, 3928, 7975, 11904, 15832, 19879, 23807, 27735, 31783, 35711, 38707, 38707, 38707, 38707, 38707 };


#ifdef MULTI_BAND_RXTX
//tables for multi band Rx/Tx boards / 01.07.2013

#define N2APB_MULTI_TABLE_SIZE 7

// switching points
long n2apb_multi_bands[N2APB_MULTI_TABLE_SIZE] = {3490000, 4000000, 8000000, 9990000, 15000000, 20990000, 30000000};

// P13 bits. D0=tx_dis, D1=RC1=pin2, D2=RC2=pin4, D3=RC3=pin3, D4=RC4=pin1
unsigned char n2apb_multi_band_bits[N2APB_MULTI_TABLE_SIZE+1] = {0x00, 0x00, 0x10, 0x02, 0x06, 0x16, 0x1A, 0x1A};
#endif


// fast set for band select bits
void set_band_bits(unsigned char band_bits)							// set band select bits
	{
	__asm__ volatile("MOV.B WREG, LATC"); 							// direct reference to J13 lower bits

	if((band_bits & TX_DISABLE) && eeprom.defval.tx_lock)			// set TX disable flag if TX Lock menu item is true and out of ham bands
		tx_disable = 1;
	else
		tx_disable = 0;
	}

#ifdef N2APB_BAND_SWITCH
// N2APB style band switch logic
void find_band_bits(unsigned long f)
	{
	int x;

#ifdef MULTI_BAND_RXTX
	unsigned char multi_band_bits;

// evaluate P13 bits
	for(x=0; x<N2APB_MULTI_TABLE_SIZE; x++)							// scan band limits table, entry on band upper edge
		{
		if((n2apb_multi_bands[x] - (long)f) > 0)					// under limit[x]
			break;													// edge found
		}

	multi_band_bits = n2apb_multi_band_bits[x];						// set P13 Multi Band board filter control bits

//	sprintf(pbuf, "\n\rf %ld, Band below %ld, sel bits 0x%.2X", f, n2apb_bands[x], multi_band_bits);
//	putst1(pbuf);

// evaluate band edges & TX inhibit limits
	for(x=0; x<N2APB_TABLE_SIZE; x++)								// scan band limits table, entry on band upper edge
		{
		if((n2apb_bands[x] - (long)f) > 0)							// under limit[x]
			break;													// edge found
		}

	if(x > N2APB_TABLE_SIZE-1)
		x = N2APB_TABLE_SIZE - 1;									// keep table index within limits

	if(f > n2apb_txinh[x])
		multi_band_bits = multi_band_bits + 1;						// set TX disable flag

	set_band_bits(multi_band_bits);									// set the digital I/O band bits
	set_pwm2_dac(n2apb_dac_val[x]);									// set band voltage DAC

#else
	unsigned char band_bits;

	for(x=0; x<N2APB_TABLE_SIZE; x++)								// scan band limits table, entry on band upper edge
		{
		if((n2apb_bands[x] - (long)f) > 0)							// under limit[x]
			break;													// edge found
		}

	if(x > N2APB_TABLE_SIZE-1)
		x = N2APB_TABLE_SIZE - 1;									// keep table index within limits

	band_bits = x << 1;												// generate P13 bits, just index*2

	if(f > n2apb_txinh[x])
		band_bits = band_bits + 1;									// set TX disable

	set_band_bits(band_bits);										// set the digital I/O band bits
	set_pwm2_dac(n2apb_dac_val[x]);									// set band voltage DAC

// debug printouts
//	sprintf(pbuf, "\n\rf %ld, Band below %ld, sel bits 0x%.2X", f, n2apb_bands[x], band_bits);
//	putst1(pbuf);
#endif
	}


#else
// programmable band switch logic
// find band segment where we are and set the P13 bits
void find_band_bits(unsigned long f)
	{
	int x;
	for(x=0; x<16; x++)												// scan band limits table, entry on band upper edge
		{
		if((cal.calval.band_limits[x] - (long)f) > 0)				// under limit[x]
			break;													// edge found
		}

	set_band_bits(cal.calval.band_bits[x]);							// set the digital I/O band bits
	set_pwm2_dac((unsigned int)cal.calval.dac_val[x] * 256);		// set band voltage DAC, stored value = pwm value / 256

// debug printouts
//	sprintf(pbuf, "\n\rf %ld, Band below %ld, sel bits 0x%.2X", f, cal.calval.band_limits[x], cal.calval.band_bits[x]);
//	putst1(pbuf);
	}
#endif


// calculate frequency offset for Yaesu type CW
void calc_yaesu_cw_offset(void)
	{
	yaesu_cw_offset = ((float)cal.calval.lo_mult * (float)eeprom.defval.cw_pitch / 8.192);	// calculate frequency from phase adder value
	}


// set band bits, calculate LO mutiplier, select Si570 or DDS and setfrequency
void calc_set_freq(unsigned long freq)
	{
	find_band_bits(freq);											// set band select bits with dial freq

	freq = (long)cal.calval.lo_mult * freq;							// LO = n * F rxtx, default n = 4

	if(yaesu_type_cw == 1)											// yaesu type CW corrections
		{
		if(eeprom.defval.mode[eeprom.defval.mem] == 2)				// 0=LSB, 1=USB, 2 = CW , 3=CWR
			freq = freq - yaesu_cw_offset;
		if(eeprom.defval.mode[eeprom.defval.mem] == 3)				// 0=LSB, 1=USB, 2 = CW , 3=CWR
			freq = freq + yaesu_cw_offset;
		}

	if(op_mode == 4 && rx_tx == 1)									// AMl & TX, calculate carrier offset 4=AMl
		freq = freq + (long)yaesu_cw_offset;						// use pre calculated offset, note lo_mult included

	if(op_mode == 5 && rx_tx == 1)									// AMu & TX, calculate carrier offset 5=AMu
		freq = freq - (long)yaesu_cw_offset;						// use pre calculated offset


	if(cal.calval.lo_type == DDS)									// DDS selected
		set_dds_freq(freq);											// set AD9850 DDS
	else
		si570_calc_set_freq(freq);									// set Si570
	}

// correct LO for Yaesu type CW
void correct_lo_for_yaesu_cw(void)
	{
	if(yaesu_type_cw == 1)			// do only if Yaesu type CW selected
		{
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
		}
	}


// set VFO min / max frequency limits
void set_vfo_min_max(void)
	{
	if(cal.calval.lo_type == DDS)									// LO type, 0 = DDS, others = Si570 types
		{
		min_vfo_freq = MIN_AD9851_FREQ / cal.calval.lo_mult;		// calculate display value
		max_vfo_freq = MAX_AD9851_FREQ / cal.calval.lo_mult;
		}
	else
		{
		min_vfo_freq = MIN_SI570_FREQ / cal.calval.lo_mult;			// calculate display value

		switch(cal.calval.lo_type)									// select Si570 typw
			{
			case SI570_180:
				max_vfo_freq = MAX_SI570_180_FREQ / cal.calval.lo_mult;
			break;

			case SI570_280:
				max_vfo_freq = MAX_SI570_280_FREQ / cal.calval.lo_mult;
			break;

			case SI570_810:
				max_vfo_freq = MAX_SI570_810_FREQ / cal.calval.lo_mult;
			break;

			case SI570_945:
				max_vfo_freq = MAX_SI570_945_FREQ / cal.calval.lo_mult;
			break;
			}
		}
	}

// force VFOs (A&B) within LO type limits
void force_vfos_within_limits(void)
	{
	long l;

	l = eeprom.defval.vfo[eeprom.defval.mem][0];			// get VFO A
	if(l < min_vfo_freq)									// compare against current limits
		l = min_vfo_freq;
	if(l > max_vfo_freq)
		l = max_vfo_freq;
	eeprom.defval.vfo[eeprom.defval.mem][0] = l;			// restore VFO A

// do also VFO B in case we have RIT on
	l = eeprom.defval.vfo[eeprom.defval.mem][1];			// get VFO B
	if(l < min_vfo_freq)									// compare against current limits
		l = min_vfo_freq;
	if(l > max_vfo_freq)
		l = max_vfo_freq;
	eeprom.defval.vfo[eeprom.defval.mem][1] = l;			// restore VFO B
	}


// collect characters from UART and display band scope when data array full
// start character 0x40 = '@'
// data anything less than 0x40, display range = 0 - 15
 
void display_band_scope(void)
	{
	unsigned char c;

	while(kbhit2() != 0)										// check PSK modem input, read all available characters
		{
		c = getch2();											// read command char

		if(c & 0x40)											// check if sync character
			band_scope_data_idx = 0;							// reset input pointer

		band_scope_data[band_scope_data_idx] = c;				// save data
		band_scope_data_idx++;

		if(band_scope_data_idx == 128)							// array full
			{
			band_scope_data_idx = 0;							// reset input pointer
			draw_bandscope((unsigned char*)band_scope_data);	// display
			}
		}
	}

// Volume pot ADC values are 0 - 4095
// head phome amp set values are 0 - 127, values below 48 are Mute values
// hp volume set = ADC / 32
// new scaling: hp volume set = ADC / 51 + 47

void vol_pot(void)
	{
	int new_vol_pot;								// new volume pot value

	if(PTT_OUT == 1)								// when TX do not addjust hp volume !, results are not good
		return;

	new_vol_pot = convert_adc12(AF_GAIN_POT);		// convert pot value

// check if moved enough
	if(	new_vol_pot > (old_vol_pot + VOL_POT_HYST) || new_vol_pot <  (old_vol_pot - VOL_POT_HYST))
		{
		old_vol_pot = new_vol_pot;					// uppdate old value
//		hp_vol = new_vol_pot >> 5;					// divide by 32
		hp_vol = (new_vol_pot / 51) + 47;			// slower but gives full pot scale

// uppdate Codec registers
		set_hp_volume(hp_vol);						// set headphone volume

// display
		display_af_gain(hp_vol);
		disp_bl_on(eeprom.defval.back_light_timer);	// lights on
		}
	}


// RF Attn pot (for now Line in Amp) ADC values are 0 - 4095
// head phome amp set values are 0 - 31
// hp volume set = ADC / 128

void rf_attn_pot(void)
	{
	int new_li_pot;									// new line in pot value

	if(PTT_OUT == 1)								// when TX do not addjust Li gain, results are not good
		return;

	new_li_pot = convert_adc12(ATTN_POT);			// convert pot value

#if 0
// check if moved enough
	if(	new_li_pot > (old_li_pot + LI_POT_HYST) || new_li_pot <  (old_li_pot - LI_POT_HYST))
		{
		old_li_pot = new_li_pot;					// uppdate old value
		li_vol = new_li_pot >> 7;					// divide by 128

// uppdate Codec registers
		set_li_volume(li_vol);						// set Line In volume

// display
		display_li_gain(li_vol);
		disp_bl_on(eeprom.defval.back_light_timer);	// lights on	
		}
#endif

// check if moved enough
	if(	new_li_pot > (old_li_pot + LI_POT_HYST) || new_li_pot <  (old_li_pot - LI_POT_HYST))
		{
		old_li_pot = new_li_pot;					// uppdate old value
		li_vol = (new_li_pot >> 5) & 0x1F;			// divide by 32 and mask
		rf_attn_bits = new_li_pot >> 10;			// divide by 128

// uppdate Codec registers
		set_li_volume(li_vol);						// set Line In volume

// set attenuator relays
		set_rf_attenuator(3 - rf_attn_bits);		// set RF attenuator relays

// display
		display_rf_attn(li_vol, rf_attn_bits);
		disp_bl_on(eeprom.defval.back_light_timer);	// lights on	
		}
	}


// keyer speed potentiometer
// ADC value / 32 = keyer speeds 1 ... 127 wpm, limited to 99
// or ADC value / 64 = keyer speeds 1 ... 63 wpm

void keyer_pot(void)
	{
	int new_k_pot;									// new volume pot value

	new_k_pot = convert_adc12(KEYER_SPD_POT);		// convert pot value

// check if moved enough
	if(	new_k_pot > (old_k_pot + KEYER_POT_HYST) || new_k_pot <  (old_k_pot - KEYER_POT_HYST))
		{
		old_k_pot = new_k_pot;						// uppdate old value
//		wpm = new_k_pot >> 5;						// divide by 32, for range 1 to 99
		wpm = new_k_pot >> 6;						// divide by 64, for range 1 to 63
		if(wpm < 1)									// Keyer speed low limit
			wpm = 1;
		if(wpm > 99)								// Keyer speed high limit
			wpm = 99;

		set_cw_period(wpm);							// set keyer speed
		display_keyer_speed(wpm);					// display new speed
		disp_bl_on(eeprom.defval.back_light_timer);	// lights on
		}
	}


// filter select potentiometer

void filter_pot(void)
	{
	int new_f_pot;									// new volume pot value

//	static int old_filter;							// old filter value
	int filter;										// selected filter 0-3.

	new_f_pot = convert_adc12(FILTER_POT);			// convert pot value, pot max = 4095

// check if moved enough
	if(	new_f_pot > (old_f_pot + FILTER_POT_HYST) || new_f_pot <  (old_f_pot - FILTER_POT_HYST))
		{
		old_f_pot = new_f_pot;						// uppdate old value
//		eeprom.defval.filter = new_f_pot;			// copy to eeprom struct, for UI	

// when changed set & display
		filter = new_f_pot / 1024;					// rough division of the pot scale
		if(filter != old_filter)					// filter changed
			{
			old_filter = filter;					// uppdate old value
//			eeprom.defval.filter = (filter * 1000);	// calculate value for display and save for display refresh
			eeprom.defval.filter = filter;			// save for display refresh
			set_pbf_coef(filter);					// set filter

//			display_filter(eeprom.defval.filter);	// display, steps
			display_underscore_filter(eeprom.defval.filter); // display alternative filter (underscore)
			disp_bl_on(eeprom.defval.back_light_timer);	// lights on
			}
		}
	}

// do all potentiometers
void all_pots(void)
	{
// first set old values out of range
	old_vol_pot = -99;	
	old_li_pot = -99;	
	old_k_pot = -99;	
	old_f_pot = -99;
// then do all pots
	vol_pot();
	rf_attn_pot();
	keyer_pot();
	filter_pot();
	}

//____________________________________

// VFO encoder switch
// Push and turn = select memory
// long push = go config

void encoder_sw(void)
	{
	int moved, x;

		if(read_switch() != 0 && PTT_OUT == 0)							// get encoder switch status, only when RX
			{
			beep(TONE_800_HZ, eeprom.defval.beep_len);					// tone, length from eeprom
			disp_bl_on(eeprom.defval.back_light_timer);					// lights on

			read_encoder(1);											// clear encoder
			moved = 0;
			while(read_switch() != 0)									// do while pressed
				{
				if(read_encoder(0) != 0)								// test if encoder moved
					{													// yes start memory selector
					moved = 1;
					while(read_switch() != 0)							// loop here as long as encoder switch pressed
						{
						x = read_encoder(1);							// read & zero encoder

						if(x !=0)										// changed
							{
							eeprom.defval.mem = eeprom.defval.mem + x;	// uppdate memory #
							beep(TONE_1200_HZ, KLICK);					// tone, length from eeprom

							if(eeprom.defval.mem < 0)					// keep within limits
								eeprom.defval.mem = 0;
							if(eeprom.defval.mem > eeprom.defval.max_mem)
								eeprom.defval.mem = eeprom.defval.max_mem;

							force_vfos_within_limits();					// keep frequency within selected LO capabilities
							display_mem_vfo();																						// display selected memory and the VFO
							display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);			// display freq
							display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);												// Mode & rx / tx state
							calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);			// also set freq
							if(rit_on == 1)
								{
								rit_on = 0;																							// remove rit when mem is changed
								meas_lcd_disp_batt_volt();																			// Remove RIT message
								}
							}
						}
					}
				}
			}
	}	

//____________________________________
//
// UI  buttons

// VFO select / Rit
void vfo_sw(void)
	{
	if(VFO_SW == 0 && PTT_OUT == 0)								// can not be changed when TX
		{
		ms_delay(DEBOUNCE_DOWN);								// start debounce delay
		if(VFO_SW == 0 && PTT_OUT == 0)							// lock button when TX on
			{
			beep(TONE_800_HZ, eeprom.defval.beep_len);			// tone, length from eeprom
			disp_bl_on(eeprom.defval.back_light_timer);			// lights on

// Long push = RIT On/Off
			long_push = LONG_PUSH;								// load SW push timer
			while(VFO_SW == 0)									// wait for release
				{
				if(long_push == 0)								// after long puch change RIT status
					{
					beep(TONE_1000_HZ, eeprom.defval.beep_len * 4);	 // tone, length from eeprom

					if(rit_on == 0)								// rit was off, turn on & copy VFO freq
						{
						rit_on = 1;								// turn RIT On

						if(eeprom.defval.vfo_sel[eeprom.defval.mem] == 0)	// VFO A selected, copy A (0) to B (1)
							{
							eeprom.defval.vfo[eeprom.defval.mem][1] = eeprom.defval.vfo[eeprom.defval.mem][0];	// A to B
							}
						else												// Copy B to A and salect A
							{
							eeprom.defval.vfo[eeprom.defval.mem][0] = eeprom.defval.vfo[eeprom.defval.mem][1];	// B to A
							eeprom.defval.vfo_sel[eeprom.defval.mem] = 0;	// select VFO A
							}
						}
					else
						rit_on = 0;								// turn RIT Off

					meas_lcd_disp_batt_volt();					// display Rit On instead of the  batt V at the LCD						
					while(VFO_SW == 0);							// wait for release
					ms_delay(DEBOUNCE_DOWN);					// some additional delay
					}
				}

			if(long_push != 0)									// was short push
				{
				eeprom.defval.vfo_sel[eeprom.defval.mem]++; 	// VFO, 0=A, 1=B
				if(eeprom.defval.vfo_sel[eeprom.defval.mem] > 1)
					eeprom.defval.vfo_sel[eeprom.defval.mem] = 0;
				}

			display_mem_vfo();																							// display selected memory and the VFO
			display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]); 				// display freq
			calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);				// also set freq

			ms_delay(DEBOUNCE_UP);																						// debounce
			}
		}
	}


// Mode select / EEPROM save
void mode_sw(void)
	{
	if(MODE_SW == 0 && PTT_OUT == 0)
		{
		ms_delay(DEBOUNCE_DOWN);								// start debounce delay
		if(MODE_SW == 0 && PTT_OUT == 0)						// lock button when TX on 
			{
			beep(TONE_800_HZ, eeprom.defval.beep_len);			// tone, length from eeprom
			disp_bl_on(eeprom.defval.back_light_timer);			// lights on

// Long push = EEPROM save
			long_push = LONG_PUSH;								// load SW push timer
			while(MODE_SW == 0)									// wait for release
				{
				if(long_push == 0)								// after long push do EEPROM user data save
					{
					beep(TONE_1000_HZ, eeprom.defval.beep_len * 4);	 // tone, length from eeprom
					display_message(MSG_CONF_SAVED);			// LCD message
					save_defaults();
					sprintf(pbuf, "User data saved to the EEPROM\n\r");
					putst1(pbuf);

					while(MODE_SW == 0);						// wait for release
					redraw_freq_info(); 						// restore frequency display before exit
					}
				}

// short push toggle operating mode
			if(long_push != 0)
				{
				eeprom.defval.mode[eeprom.defval.mem]++;						// change mode
				if(eeprom.defval.mode[eeprom.defval.mem] < 0)
					eeprom.defval.mode[eeprom.defval.mem] = 0;					// just in case something odd happens
				if(eeprom.defval.mode[eeprom.defval.mem] > 5)					// 0=LSB, 1=USB, 2=CW, 3=CWR, 4=AMl, 5=AMu
					eeprom.defval.mode[eeprom.defval.mem] = 0;					// back to LSB

// set new op_mode immediatelly
				op_mode = eeprom.defval.mode[eeprom.defval.mem]; 				// set mode for DCI etc processing
				set_iq_cor_mult();												// set I/Q correction multipliers
//				go_rx();														//stop TX if it was on
				display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);		// show new selection

				correct_lo_for_yaesu_cw();										// do only if Yaesu type CW selected
				build_bscope_markers();											// set possible CW carrier marker
				}			

			ms_delay(DEBOUNCE_UP);												// debounce
			}
		}
	}

// Tune, VFO tuning step / Tune(send RF carrier), return = 0 if no action, return = 1 if rate, return = 2 if tune
int tune_sw(void)
	{
	if(TUNE_SW == 0 && PTT_OUT == 0)							// active only when RX
		{
		ms_delay(DEBOUNCE_DOWN);								// start debounce delay
		if(TUNE_SW == 0)										// check if still pressed
			{
			beep(TONE_800_HZ, eeprom.defval.beep_len);			// tone, length from eeprom
			disp_bl_on(eeprom.defval.back_light_timer);			// lights on

// Long push start tune mode
			long_push = LONG_PUSH;								// load SW push timer
			while(TUNE_SW == 0)
				{
				if(long_push == 0)								// after long push timer expired start Tune mode
					{
					if(rx_tx == 0)								// only if we are in RX mode
						{
						beep(TONE_800_HZ, eeprom.defval.beep_len * 4);	 // Tune TX started
						go_tune();								// set tuning signal on
						}

					while(TUNE_SW == 0)							// Tune on, wait for release
						{
						select_band_scope();					// show spectrum display while tune TX on
						}

					go_rx();									// back to RX
					return 2;									// tune have been active
					}
				}

// short push advance tuning step index
			if(long_push != 0)
				{
				if(double_push != 0)							// check if double tap
					{
					eeprom.defval.step_idx = TUNING_STEPS-1;	// set to fastest
					beep(TONE_1000_HZ, eeprom.defval.beep_len * 4);	// different tone for very fast tuningtone
					}
				else
					{
					eeprom.defval.step_idx ++;						// increment tuning step ixx
					if(eeprom.defval.step_idx >= TUNING_STEPS-1)
						eeprom.defval.step_idx = 0;					// keep in range
					}

				display_tune_step(eeprom.defval.step_idx);		// display new tuning step
				}
			ms_delay(DEBOUNCE_UP);								// debounce delay
			double_push = DOUBLE_PUSH;							// set double push timer
			return 1;											// step index changed
			}
		}


	return 0;													// no action
	}

// Select Menu function / VFO lock (was Meter switch)
void menu_sw(void)
	{
	if(METER_SW == 0)											// can go Menu also when TX on
		{
		ms_delay(DEBOUNCE_DOWN);								// start debounce delay
		if(METER_SW == 0)										// check if still pressed
			{
			beep(TONE_800_HZ, eeprom.defval.beep_len);			// tone, length from eeprom
			disp_bl_on(eeprom.defval.back_light_timer);			// lights on

// Long push = toggle VFO lock
			long_push = LONG_PUSH;								// load SW push timer
			while(METER_SW == 0)								// wait for release
				{
				if(long_push == 0)								// after long push timer expired toggle VFO lock flag
					{
					beep(TONE_800_HZ, eeprom.defval.beep_len * 4);	 // Lock toggle noticed

					if(vfo_lock == 0)							// toggle
						vfo_lock = 1;
					else
						vfo_lock = 0;

//					display_tune_step(eeprom.defval.step_idx);	// uppdate display
					display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]); 				// uppdate display

					while(METER_SW == 0);						// wait for button release
					ms_delay(DEBOUNCE_DOWN);					// some additional delay
					}
				}

			if(long_push != 0)									// if this was short push
				{
				user_menu();									// run user menu
				display_tune_step(eeprom.defval.step_idx);		// uppdate display
				}

			ms_delay(DEBOUNCE_UP);								// debounce
			}
		}
	}


// set I/Q correction multipliers

// RX multipliers
void set_iq_cor_mult(void)
	{
// set I/Q correction multipliers
	if(op_mode == 0 || op_mode == 3 || op_mode == 4)			// LSB sideband or CWR selected or AMl
		{
		firl_mul = cal.calval.rx_i_mult;						// I = L, Q = R
		firr_mul = cal.calval.rx_q_mult;
		x_mul = cal.calval.rx_x_mult;
		}
	else														// USB sideband selected
		{
		firr_mul = cal.calval.rx_i_mult;						// swap L & R
		firl_mul = cal.calval.rx_q_mult;
		x_mul = cal.calval.rx_x_mult;
		}
	}

// set for TX, no FIR gain correction, gain = near 1
void set_iq_cor_unity(void)
	{
	firl_mul = 30000;											// set RX/FIR I/Q correction multipliers, unity for now
	firr_mul = 30000;
	x_mul = 0;
	}

// TX multipliers
// set for TX, multipliers from EEPROM cal table
void set_tx_iq_cor_mult(void)
	{
	txl_mul = cal.calval.tx_i_mult;								// I = L
	txr_mul = cal.calval.tx_q_mult;								// Q = R
	txx_mul = cal.calval.tx_x_mult;								// phase
	}

// set for RX, no correction, gain = 1.0
void set_tx_iq_cor_unity(void)
	{
	txl_mul = 0x7FFF;											// gain correction multiplier, Left, for txbuf0, max gain 1.0
	txr_mul = 0x7FFF;											// gain correction multiplier, Right, for txbuf1, max gain 1.0
	txx_mul = 0;												// phase correction multiplier, Left - Right mixing, no phase correction
	}


//TX / RX switching functions

// set PSK tx mode & start TX
void go_psk_tx(void)
	{
	set_agc_for_tx();											// stop AGC actions
	set_hp_volume(0);											// mute headphones
	AUDIO_SEL = 0;												// audio from PSK modem
	set_dci_input_mag(0);										// keep PSK audio in gain constant
	set_tx_iq_cor_mult();										// set for TX, multipliers from EEPROM cal table
	set_iq_cor_unity();											// set RX/FIR I/Q correction multipliers, unity for now
	dci_mode = DCI_SSB_TX;										// set TX

	if(rit_on == 1)												// RIT operation, change VFO for TX
		{
		eeprom.defval.vfo_sel[eeprom.defval.mem] = 1;			// select VFO B for TX
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display new freq
		display_mem_vfo();	
		}

// keep Line in as audio input
	if(tx_disable == 0)											// TX enabled
		{
		set_li_volume(PSK_TX_LI_GAIN);							// set Line In gain gor PSK TX
		PTT_OUT = 1;											// soft Rock PTT on
		rx_tx = 2;												// display PSK TX
		}
	else
		{
		rx_tx = 9;												// display TX Error
		beep(TONE_2000_HZ, eeprom.defval.beep_len * 4);	 		// give error beep
		}

	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);
	}


// set Keyer tx mode & start TX
void go_keyer_tx(void)
	{
// TX hardware handling happens in the keyer IRQ code
	set_agc_for_tx();											// stop AGC actions
	set_tx_iq_cor_mult();										// set for TX, multipliers from EEPROM cal table

	if(rit_on == 1)												// RIT operation, change VFO for TX
		{
		eeprom.defval.vfo_sel[eeprom.defval.mem] = 1;			// select VFO B for TX
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display new freq
		display_mem_vfo();	
		}

//	set_hp_volume(0);											// mute headphones, do codec config write
	set_hp_volume(eeprom.defval.cw_sidetone_level);				// set earphone sidetone level

	if(tx_disable == 0)											// TX enabled
		{
		rx_tx = 3;												// display TXcw
		}
	else
		{
		rx_tx = 9;												// display TX Error
//		beep(TONE_2000_HZ, eeprom.defval.beep_len * 4);	 		// give error beep
		}

	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);
	}


// set SSB or AM tx mode & start TX
void go_tx(void)
	{
	set_hp_volume(0);											// mute headphones

if(tx_disable == 0)												// TX enabled
		{
		PTT_OUT = 1;											// soft Rock PTT on
		rx_tx = 1;												// display TX
		}
	else
		{
		rx_tx = 9;												// display TX Error
		beep(TONE_2000_HZ, eeprom.defval.beep_len * 4);	 		// give error beep
		return;													// do nothing more
		}

	if(op_mode == 4 || op_mode == 5)							// AM  modes ?, correct carrier frequency with CW pitch
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);

	if(op_mode == 4 || op_mode == 5)							// AM  modes ?
		dci_mode = DCI_AM_TX;									// set AM TX
	else
		dci_mode = DCI_SSB_TX;									// set SSB TX

	if(rit_on == 1 && (op_mode == 0 || op_mode == 1))			// SSB RIT operation, change VFO for TX
		{
		eeprom.defval.vfo_sel[eeprom.defval.mem] = 1;			// select VFO B for TX
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display new freq
		display_mem_vfo();	
		}

	set_agc_for_tx();											// stop AGC actions
	set_tx_iq_cor_mult();										// set for TX, multipliers from EEPROM cal table
	set_iq_cor_unity();											// set RX/FIR I/Q correction multipliers, unity for now

	set_dci_input_mag(0);										// keep microphone gain constant

	tlv_aapc = TLV_AAPC | ( TLV_DAC | TLV_INSEL | TLV_MICB); 	// select MIC, +20dB
//	tlv_aapc = TLV_AAPC | ( TLV_DAC | TLV_INSEL ); 				// select MIC
	write_codec_spi(tlv_aapc);

	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);
	}

// go back to RX
void go_rx(void)
	{
	PTT_OUT = 0;												// RF Unit PTT off
	rx_tx = 0;													// RX status

	if(op_mode == 4 || op_mode == 5)							// AM  modes ?
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);	// reset RX frequency
	else
		{
		if(rit_on == 1)											// SSB RIT operation, change VFO for TX
			{
			eeprom.defval.vfo_sel[eeprom.defval.mem] = 0;		// select VFO B for TX
			calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
			display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display new freq
			display_mem_vfo();
			}
		}

//  audio blackout after TX -> RX switch
	if(op_mode == 2 || op_mode == 3 || AUDIO_SEL == 0)			// short delay if CW or CWR selected or PSK TX
		mute_ctr = RX_MUTE_BASE;
	else														// long delay if Mic/Line need switvhing
		mute_ctr = RX_MUTE_BASE + (eeprom.defval.rx_mute_var * eeprom.defval.codec_mag);	// hide codec bias match problem when Mic amp is switched to Line amp

	set_dci_input_mag(eeprom.defval.codec_mag);					// set codec input magnitude multiplier as it was before TX

// set I/Q correction multipliers
	set_iq_cor_mult();											// RX correction multipliers from EEPROM calibration table
	set_tx_iq_cor_unity();										// TX correction, Codec output to unity gain 1.0

	AUDIO_SEL = 1;												// soft rock audio

	tlv_aapc = TLV_AAPC | ( TLV_DAC | TLV_MICM );				// select LINE L/R, mute Mic
	write_codec_spi(tlv_aapc);

	dci_mode = DCI_SSB_RX;										// set RX

	set_hp_volume(hp_vol);										// restore  hp volume controls

	if(rx_tx == 2)												// check if we come back from PSK TX, if not just save some time
		set_li_volume(li_vol);									// set RF Attn value to the Line In amp
													
	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);	// display RX

	set_agc_for_rx();											// set AGC for RX
	}

// start tune carrier TX
void go_tune(void)
	{
	set_agc_for_tx();											// stop AGC actions
	set_hp_volume(0);											// mute headphones

	if(rit_on == 1)												// RIT operation, change VFO for TX
		{
		eeprom.defval.vfo_sel[eeprom.defval.mem] = 1;			// select VFO B for TX
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display new freq
		display_mem_vfo();	
		}

	set_tx_iq_cor_mult();										// set for TX, multipliers from EEPROM cal table
	dci_mode = DCI_TUNE_TONE;									//switch audio source to soft DDS

	if(tx_disable == 0)											// TX enabled
		{
		PTT_OUT = 1;											// soft Rock PTT on
		rx_tx = 4;
		}
	else
		{
		rx_tx = 9;												// display TX Error
		beep(TONE_2000_HZ, eeprom.defval.beep_len * 4);	 		// give error beep
		}

	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);
	}



// Tune VFO freq
// tune frequency if encoder moved
void tune_freq(void)
	{
	int x;
	long l;													// signed long for compare, max value = 2 147 483 647

	x = read_encoder(1);									// read & clear encoder
	if(x != 0 && vfo_lock == 0 && PTT_OUT == 0)				// moved & VFO not locked & RX
		{

// tune
		l = eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]];			// get VFO
		l = l + (x * vfo_tuning_steps[eeprom.defval.step_idx]);										// addjust display frequency

// check limits
		if(l < min_vfo_freq)		// keep the display within limits
			l = min_vfo_freq;
		if(l > max_vfo_freq)
			l = max_vfo_freq;

		eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]] = l;			// restore VFO


// display
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);					// display new freq
		display_tune_step(eeprom.defval.step_idx);																		// display tuning step
		calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);

// display
		draw_dummy_flag = 1;								// trigger dummy bandscope draw
		disp_bl_on(eeprom.defval.back_light_timer);			// lights on
		}
	}


// select & display band scope
void select_band_scope(void)
	{

	calc_fft_bandscope();			// only alternative now, changed 05.05.2011

#if 0
		switch(eeprom.defval.bandscope_mode)
			{
			case 0:					// dummy data for scope off air demo
				if(draw_dummy_flag)
					{
					draw_dummy_flag = 0;
					draw_dummy_scope();
					}
			break;

			case 1:					// UART 2 data
				display_band_scope();
			break;

			case 2:					// local calculations
				calc_fft_bandscope();
			break;
			}
#endif
	}

//______________________________________________
//
// RX / TX transition logic
//

void rx_tx_logic(void)
		{
// set current op_mode
			op_mode = eeprom.defval.mode[eeprom.defval.mem]; // set mode for DCI etc processing

			if(op_mode == 0 || op_mode == 1 || op_mode == 4 || op_mode == 5)			// LSB or USB or AM modes selected
				{

// TX / RX switch
				mic_ptt = MIC_PTT;							// get copy of Mic PTT switch
				if(mic_old_ptt != mic_ptt)					// state changed
					{
					mic_old_ptt = mic_ptt;					// uppdate old status

					if(mic_ptt == 0)						// TX
						{
						go_tx();
						psk_old_ptt = 1;					// reset PSK tx flag if it was on
						}
				else										// RX
						{
						go_rx();
						}
					}

// PSK TX
				if(mic_ptt != 0)								// microphone have precedence
					{
					psk_ptt = PSK_PTT;							// get copy of PSK PTT swits
					if(psk_old_ptt != psk_ptt)					// state changed
						{
						psk_old_ptt = psk_ptt;					// uppdate old status

						if(psk_ptt == 0)						// start PSK TX
							{
							go_psk_tx();
							}
						else									// RX
							{
							go_rx();
							}
						}
					}
				}


// keyer TX
			if(keyer_tx_req == 1 && (op_mode == 2 || op_mode == 3))		// keyer wants to transmit and mode = CW or CWR
				{
				if(keyer_tx_on == 0)					// was off
					{
					go_keyer_tx();
					keyer_tx_on = 1;					// TX granted
					}
				}

			else				
				{
				if(keyer_tx_on == 1)					// was on
					{
					go_rx();
					keyer_tx_on = 0;					// TX status removed
					}
				}
		}	


// main __________________________________________________________________________________

int main(void)
	{
	long ltemp;						// agc calc temp variable

// set general I/O
	TRISA = TRISA_INIT;				// 
	PORTA = PORTA_INIT;

	TRISB = TRISB_INIT;				// 
	PORTB = PORTB_INIT;

	TRISC = TRISC_INIT;				//
	PORTC = PORTC_INIT;

	TRISD = TRISD_INIT;				//
	PORTD = PORTD_INIT;

	TRISF = TRISF_INIT;				//
	PORTF = PORTF_INIT;

	TRISG = TRISG_INIT;				//
	PORTG = PORTG_INIT;

// Init PLL and wait for PLL lock
    LD3 = 1;						// All on on, SW started indication
    LD4 = 1;


// set PLL divisor
// FCY = Fosc / 2
// FCY must be a "baud rate clock", integer multiple of 1,8432MHz

// 29,4912 MHz FCY setup, CPU current 65mA
#if 0
	PLLFBDbits.PLLDIV	=	30;		// Xtal(7,3728MHz) / 2 * 32 = 117,9648MHz
// set PRE and POST scalers
	CLKDIVbits.PLLPRE	=	0;		// N1 = 2
	CLKDIVbits.PLLPOST	=	0;		// N2 = 2, 117,9648MHz / 2 / 2 = 29,4912 MHz FCY
#endif

// 38,7072 MHz FCY setup, CPU current 78mA
//#if 0
	PLLFBDbits.PLLDIV	=	61;		// Xtal(7,3728MHz) / 3 * 63 = 154,8288MHz
// set PRE and POST scalers
	CLKDIVbits.PLLPRE	=	1;		// N1 = 3
	CLKDIVbits.PLLPOST	=	0;		// N2 = 2, 154,8288MHz / 2 / 2 = 38,7072 MHz FCY
//#endif

// common clock setup
	CLKDIVbits.FRCDIV	=	4;
	CLKDIVbits.DOZEN	=	0;
	CLKDIVbits.DOZE		=	0;
	CLKDIVbits.ROI		=	0;

	while ( !OSCCONbits.LOCK );		// wait for PLL to lock


// Init UARTs
	init_uart1(9600);				// init terminal UART, speed = 9600 
	init_uart2(9600);				// init PSK UART

// Say something to the terminal
	sprintf(pbuf, "      \n\r\n\rSDR Cube Start");
	putst1(pbuf);					// Terminal UART

// System clock
	sprintf(pbuf, "\n\rCPU speed, FCY = %ld Hz\n\r", FCY);
	putst1(pbuf);

// Start ADC, must be done to get Port B I/O working
	init_adc12();
	sprintf(pbuf, "ADC system started\n\r");
	putst1(pbuf);

// we may need LCD already here
	Init_LCD();						// init LCD
	Clear_Display(0);
	disp_bl_on(0);					// lights on
	sprintf(pbuf, "LCD init\n\r");
	putst1(pbuf);

//
	display_start_up_screen();		// display start up screen, ver & date etc info

// EEPROM interface
	init_I2C_1();					// I2C CH1 used foe EEPROM communications
//	sprintf(pbuf, "24LC256 EEPROM I/O init, BRG 0x%.4X, CON 0x%.4X, STAT 0x%.4X\n\r", I2C2BRG, I2C2CON, I2C2STAT); // print debug data
	sprintf(pbuf, "24LC256 EEPROM I/O init\n\r");
	putst1(pbuf);

// Read EEPROM user & config data
	d_cs = read_defaults();			// user data
	c_cs = read_calval();			// calibration values

	if(d_cs != eeprom.defval.d_csum || c_cs != cal.calval.c_csum)		// verify checksums
		{
		sprintf(pbuf, "\n\r\n\r***** EEPROM checksum error, 0x%.4X, 0x%.4X, Reset to Factory Defaults ****\n\r\n\r", d_cs, c_cs);
		putst1(pbuf);
	
		set_factory_defaults();		// set factory defaults
		save_defaults();			// save user data
		save_calval();				// save calibration values to the EEPROM
		}
	else
		{
		sprintf(pbuf, "User data restored successfully\n\r");
		putst1(pbuf);
		}

	LD3 = 0;						// EEPROM read Ok. turn LD3 off

// Start timer system
	init_timers_pwm();
	sprintf(pbuf, "Timer system started\n\r");
	putst1(pbuf);


// Check if Factory Default config is requested by the user, Mode(Save) button pressed
	if(MODE_SW == 0)
		{
		beep(TONE_2000_HZ, 1000);
		display_message(MSG_FACT_CONFIG);	// LCD message, Factory Defaults
		sprintf(pbuf, "\n\r\n\r***** EEPROM Factory Defaults set by user *****\n\r\n\r");
		putst1(pbuf);
		set_factory_defaults();		// set factory defaults
		save_defaults();			// save user data
		save_calval();				// save calibration values to the EEPROM
		while(MODE_SW == 0);		// wait as long button pressed
		ms_delay(2000);				// and little more
		}

// set process variables after EEPROM read
	op_mode = eeprom.defval.mode[eeprom.defval.mem];		// set mode for DCI etc processing

// Init & start DSP filters & codec
	init_audio_dds();			// init audio soft DDS
	ph_adder = 8193;			// 1000 Hz, fs = 8kHz
	init_fir();					// init filter tables
	set_mic_coef(eeprom.defval.mic_filter);	// set microphone filter
	sprintf(pbuf, "DSP filters initialized\n\r");
	putst1(pbuf);

	init_codec_spi();				// init codec control port
	init_tlv320();					// init codec
	init_dci();						// init dsPIC DCI port
	_DCIIF = 0;
	_DCIIE = 1;						// enable DCI IRQ

	AUDIO_SEL = 1;					// soft rock audio selected

	sprintf(pbuf, "Codec started\n\r");
	putst1(pbuf);

	disp_bl_on(eeprom.defval.back_light_timer);		// switch lights on

//	display_start_up_screen();						// display start up screen, ver & date etc info

	ms_delay(3000);									// show start up delay
	build_bscope_markers();							// set band scope marker line
	display_run_screen();							// draw basic objects
	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);

// give start beep
	beep(TONE_800_HZ, 100);							// 800Hz beep

// start  RX
	go_rx();										// start RX
	set_audio_delay(eeprom.defval.binaural);		// set new Binaural audio delay
	set_dci_input_mag(eeprom.defval.codec_mag);		// set codec input magnitude multiplier, powers of two
	set_rf_attenuator(eeprom.defval.rf_attn);		// set RF attenuator relays

// start Si570 I2C 
	init_I2C_2();
	sprintf(pbuf, "Si570 I2C init\n\r");
	putst1(pbuf);

	if(cal.calval.lo_type > DDS)						// init Si570 only if selected and chip installed
		{
// init Si570 frequency calculations
		si570_read_config();						// read Si570 factory config
		si570_set_xtal(cal.calval.si570_ref_osc);	// set Si570 XTAL calibration value from the EEPROM
		si570_init_done = 1;						// flag neded for possible runtime LO change
		}

// set DDS ref clock value
	set_dds_clk(cal.calval.dds_ref_osc);			// set AD9850 DDS ref osc calibration value from the EEPROM

// start DDS
	dds_init();										// does not matter if we have chip installed or not
	set_dds_freq(1000000);
	set_dds_freq(1000000);	// test
	sprintf(pbuf, "DDS started\n\r");
	putst1(pbuf);

	LD4 = 0;										// Si570 / DDS start up ok

// set display frequency tuning limits
	set_vfo_min_max();
	force_vfos_within_limits();

// display selected LO type
	if(cal.calval.lo_type == DDS)
		sprintf(pbuf, "AD9850 LO selected\n\r");
	else
		{
		sprintf(pbuf, "Si570 LO selected\n\r");
		set_dds_freq(0);							// also stop DDS when si570 selected
		}		
	putst1(pbuf);

// set CW parameters
	// keyer speed comes from pot
	if(eeprom.defval.cw_pitch < CW_PHASE_ADDER_LOW)	// test just for SW uppgrade compatibility
		eeprom.defval.cw_pitch = CW_PHASE_ADDER;	// force to default
	set_audio_dds_pha(eeprom.defval.cw_pitch);		// set RF tone pitch, phase adder value
//	set_audio_dds_freq(eeprom.defval.cw_pitch);		// set RF tone pitch
	set_cw_sidetone(eeprom.defval.cw_sidetone);		// set sidetone pitch
	calc_yaesu_cw_offset();							// calculate Yaesu style CW LO offset

// clear encoder & set operating frequency
	read_encoder(1);																				// clear encoder
	display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);	// display freq
	calc_set_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);	// set Si570 or DDS LO

// set I / Q amplitude & phase correction parameters
	set_iq_cor_mult();								// set RX pultipliesr

// change to user baud rate
	sprintf(pbuf, "Change Terminal UART to user baud rate %ld\n\r", baud_rates[eeprom.defval.br1]);
	putst1(pbuf);
	sprintf(pbuf, "Change PSK Modem UART to user baud rate %ld\n\r", baud_rates[eeprom.defval.br2]);
	putst1(pbuf);

		set_uart1_baud_rate(baud_rates[eeprom.defval.br1]);	// change Terminal UART baud rate
		set_uart2_baud_rate(baud_rates[eeprom.defval.br2]);	// change PSK UART baud rate

	sprintf(pbuf, "\n\rSDR Cube UART user baud rate = %ld\n\r", baud_rates[eeprom.defval.br1]);
	putst1(pbuf);

// start bandscope
	init_FFT();												// init tables
	init_window_mult(eeprom.defval.fft_window);				// set window multipliers
	start_fft();											// start sample collection

// Say hello
	sprintf(pbuf, "\n\rSDR Cube SW started");
	putst1(pbuf);

	sprintf(pbuf, "\n\rCopyright (C) 2010 - 2013 by George Heron N2APB / Juha Niinikoski OH2NLT,");
	putst1(pbuf);
	sprintf(pbuf, "\n\rMidnight Design Solutions, LLC.");
	putst1(pbuf);

	sprintf(pbuf, "\n\rVersion: ");
	putst1(pbuf);
	putst1((char*)&ver);
	sprintf(pbuf, "\n\rDate:    ");
	putst1(pbuf);
	putst1((char*)&date);
	sprintf(pbuf, "\n\r");
	putst1(pbuf);

// just for version upgrade
	if(eeprom.defval.am_gain == 0)	
		eeprom.defval.am_gain = 128;						// AM RF gain to default if not set yet

//_______________________________________________________________
//
// Main forever loop
	sprintf(pbuf, "Starting Main\n\r");
	putst1(pbuf);
	seconds = 0;

	for(;;)
		{
// DSP board LEDS
//
// LD3 Red is used as ADC overload indicator, DCI module drives it
//
// LD4 SW heart beat
//
	if(seconds & 1)
		LD4 = 1;
	else
		LD4 = 0;

// loop performance test

// Main idle loop about 750..800us
// Band scope is calculated on every 128 samples (16ms). Calculations take about 4ms
// S-beter is calculated on every 100ms. Calculations take about 800us dBm, 700uS S-meter
// IRQ code at SSB RX about 17us 


//	J13_2 = !J13_2;
	TEST_PB6 = !TEST_PB6;			// toggle PGC signal at P6-5

//____________________________________

// User interface


// UI potentiometers
		vol_pot();					// addjust HP volume
		rf_attn_pot();				// addjust Line in amp for now
		keyer_pot();				// addjust keyer speed
		filter_pot();				// filter pot test / display

// measure & display battery voltage
		meas_lcd_disp_batt_volt();

// Encoder & UI switches

// check encoder & tune
		tune_freq();

// check encoder switch
		encoder_sw();

// check other User Interface switches
		vfo_sw();
		mode_sw();
		tune_sw();
		menu_sw();

//RX / TX transition logic
		rx_tx_logic();

// run debug terminal
		terminal_debug();

//calculate & display band scope
		select_band_scope();

//___________________________________

// Calculate S-meter
	if(sm_sample_rdy == 1)
		{
		__asm__ volatile("disi #7"); 
		iq_energy_2 = iq_energy;		// make IRQ protected copy
		sm_sample_rdy = 0;				// reset sample ready flag

	if(rx_tx == 0)					// calculate & display only when RX
		{

// convert to decibels & correct with Codec Mag & Li Attn values
		raw_db = 10.0 * log10f(iq_energy_2);						// change to log scale, note our input is already IQ^2

// prevent meter reversing when low signal and high attenuator value (- dB)
		if(rf_attn_cur_val < 0.0)										// if attenuation (- dB)
			{
			if(raw_db > fabsf(rf_attn_cur_val))						// simple merer reversing lock
				raw_db = raw_db - rf_attn_cur_val;					// correct with RF attenuator & Codec Li amp gain
			}
		else														// gain (+dB)
			raw_db = raw_db - rf_attn_cur_val;						// correct with RF attenuator & Codec Li amp gain

//		raw_db = raw_db - (float)(eeprom.defval.codec_mag * 6);		// correct with codec mag selection, 6dB / step, EEPROM value
		raw_db = raw_db - (float)(mag_sel * 6);						// correct with codec mag selection, 6dB / step, RUN value
		raw_db = raw_db + (float)fir_gain_cor;						// correct with fir gain setting

//		sprintf(pbuf, "%3.1f", raw_db);
//		Write_String_LCD (FILTER_LINE, FILTER_COL, pbuf);			// old filter graphics position

// scale raw value
	raw_db = raw_db - cal.calval.mtr_cal;							// subtract S9 0ffset, 0 = -73dBm

// select meter type
	if(eeprom.defval.mtr_type != 0)									// dBm scale selected
		{															// Display dB scale, use S9_REF as reference
		raw_db = raw_db - S9_dBm;									// offset to dBm scale (-73dBm / S9)
		sprintf(pbuf, "MT:%4.0fdBm", raw_db);						// show in dBm scale
//		sprintf(pbuf, "MT:%3.1fdBm ", raw_db);						// show in dBm scale, decimals for testing & cal
		Write_String_LCD (FILTER_LINE, FILTER_COL, pbuf);			// old filter graphics position
		}

	else															// S-meter scale selected
		{															// Display S-meter
		agc_units = raw_db;

		if(agc_units > -1)											// over S9
			{
			agc_units = (agc_units / 5) * 5;						// make steps coarcer, 5dB steps
			if(agc_units == 0)										// plain S9
				sprintf(pbuf, "MTR: S9   ");
			else													// S9+
				sprintf(pbuf, "MTR: S9+%2d", agc_units);			// agc_units are directly S9+ dBs
			}

		else
			{
			s_units = 9;											// start from S1
			if(agc_units > -6)
				s_units = 8;
			else
				{
				if(agc_units > -12)
					s_units = 7;
				else
					{
					if(agc_units > -18)
					s_units = 6;
					else
						{
						if(agc_units > -24)
							s_units = 5;
						else
							{
							if(agc_units > -30)
								s_units = 4;
							else
								{
								if(agc_units > -36)
									s_units = 3;
								else
									{
									if(agc_units > -42)
										s_units = 2;
									else
										{
										s_units = 1;
										}
									}
								}
							}
						}
					}
				}
			sprintf(pbuf, "MTR: S%1d   ", s_units);					// S1 - S9
			}

			Write_String_LCD (FILTER_LINE, FILTER_COL, pbuf);		// old filter graphics position
		}							// end S-meter
		}

	else							// when Tx do not show the meter
		{
		sprintf(pbuf, "MTR: -----");
		Write_String_LCD (FILTER_LINE, FILTER_COL, pbuf);
		}

		}							// sm_sample_rdy if()

//___________________________________

// Calculate & set AGC

if(rx_tx == 0)					// calculate AGC only when RX
	{		
	if(agc_sample_rdy == 1)
		{
		agc_sample_rdy = 0;

// AGC on and calculations enabled
		if(eeprom.defval.agc_state == 1 && agc_on != 0)						// calculate AGC allowed
			{
// prepare for calculations
			agc_input_val = iq_abs_val;										// make IRQ protected copy
			agc_set_val = 65535;											// use full gain bellow AGC treshold

// run attack timer
			if(agc_attack_timer != 0)										// down count attac timer(counter)
				agc_attack_timer--;


// check if over the AGC knee
			if(agc_input_val > eeprom.defval.agc_knee)						// calculate only if greater than KNEE (treshold value)
				{
				agc_input_val = agc_input_val - eeprom.defval.agc_knee;		// subtract AGC knee offset (default val 210)

// AGC slope, calculate AGC input value (RF power)
				agc_input_val = agc_input_val * eeprom.defval.agc_slope;	// agc_slope value 2 = gain 1

// input value, Larger number = larger signal
				if(agc_input_val > 65535)
					agc_input_val = 65535;


// set attack timer
				if(agc_input_val > agc_attack_val)
					{
					agc_attack_val = agc_input_val;
					agc_attack_timer = eeprom.defval.agc_attac;				// set attack timer for new signal rise
					}
				else
					{
					if(agc_attack_timer == 0)
						agc_attack_val = agc_peak_val;						// use decaying peak value as new reference
					}


// peak hold, set hang time
				if(agc_input_val > agc_peak_val)
					{
					if(agc_attack_timer == 0)								// attack timer expired
						{
						agc_peak_val = agc_input_val;						// set new peak value, current input
						agc_hang_timer = eeprom.defval.agc_hang;			// hang timer (ms)
						}
					}
				}

// bellow the AGC knee value, no gain adjustment calculations performed, hang timer can be still running
			else
				{
				agc_attack_timer = 0;										// reset attack timer
				agc_attack_val = 0;
				}

// release slope, do if hang timer expired
				if(agc_hang_timer == 0)										// release slope
					{
					ltemp = agc_peak_val;									// can be 65535
					ltemp = ltemp - eeprom.defval.agc_release;				// release speed

					if(ltemp < 0)											// drop down to max gain
						ltemp = 0;

					agc_peak_val = ltemp;									// decrease peak values with release slope
					}

// calculate AGC system set value & set gain
			if(agc_attack_timer != 0)										// during attack delay
				{
				agc_set_val = agc_set_val - agc_attack_val;					// use attack period peak value
				agc_action_indicator = agc_attack_val >> 11;				// do simple scaling to 0 .. 31, meter range 0 .. 31)
				}

			else
				{
				agc_set_val = agc_set_val - agc_peak_val;					// when set / hang time ggoing use peak value
				agc_action_indicator = agc_peak_val >> 11;					// do simple scaling to 0 .. 31, meter range 0 .. 31)
				}

// set gain
			set_agc_gain(agc_set_val);

// action indicator
			display_bar_meter(agc_action_indicator);					// display indicator, only when AGC on
			}

// AGC off
// do nothing
		}								// AGC_sample_rdy if()

	}									// if(rx_tx == 0)
	else
		agc_sample_rdy = 0;

//__________________________________________________________
		}							// end for(;;) main loop
	}								// end main
