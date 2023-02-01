//
// SDR_Cube Display & UI functions
// OH2NLT / Juha Niinikoski 21.08.2010
//
// MPLAB v8.40, Microchip C30 v3.20
//

// SDR Cube graphic display & User Interface functions

// Strange cordinate system is from NUE-PSK display driver
// X = line#, Y = pixel position in the line(pixel column)
// Digit blancking added for tuning resolution indication 02.11.2010


/////////////////////////
//
// George Test version
//
//////////////////////////


// Version history
// Band scope center marker added 13.09.2010


// include files
#include <stdio.h>
#include "sdr-cube.h"							// board & processor specific definitions
#include "cube_globals.h"						// global data
#include "adc12.h"								// ADS system & scaling functions
#include "NUE_PSK_LCD_single_sdr.h"				// Graphic LCD driver
#include "SDR_Cube_Display_UI.h"				// Display & User interface definitions & constants

// rf attenuator steps
const char rf_attn_values[4] = {-16, -10, -6, -0};	// attenuator values (dB)

// bandscope markers, 62,5Hz / pixel

const unsigned char b_scope_marker[128] = {
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	// -4000Hz
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	// -3000Hz
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,	// -2000Hz
1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,	// -1000Hz
									// 0 Hz
7,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,	// 1000Hz
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,	// 2000Hz
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,	// 3000Hz
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1		// 4000Hz
};


// dummy bandscope data, 0 .. 15

const unsigned char dummy_scope[128] = {
0, 0, 1, 2, 3, 4, 9, 10,
4, 3, 2, 1, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,

0, 0, 3, 4, 5, 3, 2, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
1, 4, 2, 7, 9, 6, 11, 15,

11, 6, 9, 7, 2, 4, 1, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 7, 0, 0, 9, 0, 0, 0,

0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 1, 2, 5, 7, 10, 7,
4, 1, 0, 0, 0, 0, 0, 0,
};

// spectrum display bars
const unsigned char S_BARS[8] = {0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF};	// verticaly inverted table
#define S_EMPTY 0x00		// empty column
#define ZERO_MARKER 0x03	// band scope zero marker, two uppermost pixels

// bandscope spectrum display
// Note ! data buffer should be stable during picture write (high & low lines)
//

#if 0
// zero marker only
void draw_bandscope(unsigned char * sdata)		// sdata = pointer to the spectrum data
	{
	unsigned char* s;
	int sample;
	int x;

	s = sdata;											// set working pointer
	Set_Display_XYAddr(S_DISP_LINE, S_DISP_COL);		// upper line
	for(x=0; x< S_DISP_LEN; x++)						// set draw buffers
		{
		sample = *s;									// get sample
		if(sample > 15)
			sample = 15;								// saturate

		if(sample > 7)
			{
			sample = sample - 8;						// line offset
			if(x == 63 || x == 64)						// center marker place ?
				Write_Display_Data((S_BARS[sample] | ZERO_MARKER), 0);	// get higher line pixels, add marker pixels
			else
				Write_Display_Data(S_BARS[sample], 0);	// get higher line pixels
			}
		else
			{
			if(x == 63 || x == 64)						// center marker place ?
				Write_Display_Data(ZERO_MARKER, 0);		// center marker
			else
				Write_Display_Data(S_EMPTY, 0);			// all off
			}
		s++;
		}

	s = sdata;											// reload working pointer
	Set_Display_XYAddr(S_DISP_LINE+1, S_DISP_COL);		// lower line
	for(x=0; x< S_DISP_LEN; x++)						// set draw buffers
		{
		sample = *s;									// get sample
		if(sample > 15)
			sample = 15;								// saturate
		if(sample > 7)
			Write_Display_Data(S_BARS[7], 0);			// all on			
		else
			Write_Display_Data( S_BARS[sample], 0);		// get lower line pixels

		s++;
		}
	}
#endif

// Zero marker and 1000Hz dots
void draw_bandscope(unsigned char * sdata)		// sdata = pointer to the spectrum data
	{
	unsigned char* s;
	int sample;
	int x;

	s = sdata;											// set working pointer
	Set_Display_XYAddr(S_DISP_LINE, S_DISP_COL);		// upper line
	for(x=0; x< S_DISP_LEN; x++)						// set draw buffers
		{
		sample = *s;									// get sample
		if(sample > 15)
			sample = 15;								// saturate

		if(sample > 7)
			{
			sample = sample - 8;						// line offset
			Write_Display_Data((S_BARS[sample] | b_scope_marker[x]), 0); // get higher line pixels & add marker dots
			}
		else
			Write_Display_Data(b_scope_marker[x], 0);	// all off / marker dots
		s++;
		}

	s = sdata;											// reload working pointer
	Set_Display_XYAddr(S_DISP_LINE+1, S_DISP_COL);		// lower line
	for(x=0; x< S_DISP_LEN; x++)						// set draw buffers
		{
		sample = *s;									// get sample
		if(sample > 15)
			sample = 15;								// saturate
		if(sample > 7)
			Write_Display_Data(S_BARS[7], 0);			// all on			
		else
			Write_Display_Data( S_BARS[sample], 0);		// get lower line pixels

		s++;
		}
	}


// Test, draw dummy band scope from fixeed data
void draw_dummy_scope(void)
	{
	draw_bandscope((unsigned char*)dummy_scope);
	}


// Display start up screen
void display_start_up_screen(void)
	{
	Clear_Display(0);
	Write_Large_String_LCD (2,6," SDR Cube");
	Write_String_LCD (4,7, (char *)ver);
	Write_String_LCD (4,60, (char *)date);
	Write_String_LCD (5,0," Copyright 2010      ");
	Write_String_LCD (6,0," J. Niinikoski OH2NLT");
	Write_String_LCD (7,0," G. Heron      N2APB ");
	}



// display tuning step indicator
void display_tune_step(int step)
	{
	if(vfo_lock == 0)			// VFO in use
		{
		switch(step)
			{
			case 0:
				Write_String_LCD (T_STEP_LINE, T_STEP_COL,"slo");
			break;

			case 1:
				Write_String_LCD (T_STEP_LINE, T_STEP_COL,"med");
			break;

			case 2:
				Write_String_LCD (T_STEP_LINE, T_STEP_COL,"fst");
			break;

			default:
				Write_String_LCD (T_STEP_LINE, T_STEP_COL,"???");
			break;
			}
		display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);		// display current freq
		}
	else														// VFO locked, display it
		{
		Write_String_LCD (T_STEP_LINE, T_STEP_COL,"Lck");
		}
	}


// Memory # & VFO indicator, data from the EEPROM structures
void display_mem_vfo(void)
	{
//	eeprom.defval.mem										// memory #
//	eeprom.defval.vfo_sel[eeprom.defval.mem] 				// VFO, 0=A, 1=B

	sprintf(pbuf, "VFO:%.2d,", eeprom.defval.mem); 			// header & Mem #
	Write_String_LCD (VFO_LINE, VFO_COL, pbuf);

	if(eeprom.defval.vfo_sel[eeprom.defval.mem] == 0)		// select VFO letter
		Write_Char('A', 0);
	else
		Write_Char('B', 0);
	}


// display Mode and TX state
void display_mode(int mode, int rxtx)
	{
	switch(mode)
		{
		case 0:	// LSB
			Write_String_LCD (MODE_LINE, MODE_COL,"LSB:");
		break;

		case 1:	// USB
			Write_String_LCD (MODE_LINE, MODE_COL,"USB:");
		break;

		case 2:	// CW
			Write_String_LCD (MODE_LINE, MODE_COL,"CW :");
		break;

		case 3:	// AM
			Write_String_LCD (MODE_LINE, MODE_COL,"AM :");
		break;

		default: // ?
			Write_String_LCD (MODE_LINE, MODE_COL,"???:");
		break;
		}


	switch(rxtx)
		{
		case 0:	// RX
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"RX  ");
		break;

		case 1:	// TX
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"TX  ");
		break;

		case 2:	// TX PSK
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"PSKx");
		break;

		case 3:	// TX CW
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"TXcw");
		break;

		case 4:	// Tune
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"Tune");
		break;

		default: // ?
			Write_String_LCD (MODE_LINE, MODE_COL + 4*6,"????");
		break;
		}
	}



// Draw graphic filter (or other) indicator __|----|_____

#define FCHR_L 0x80		// low line
#define FCHR_R 0xFC		// rise
#define FCHR_H 0x04		// high line
#define F_SYM_LEN 32	// filter symbol length, pixels

// draw filter symbol
// len = high line length, 0 ... (F_SYM_LEN - 5)
void draw_filter_graph(unsigned char len)
	{
	int x;
	int l;

	if(len > (F_SYM_LEN - 6))			// keep in bounds
		len = (F_SYM_LEN - 6);

	l = len;
	Write_String_LCD (FILTER_LINE, FILTER_COL, "Filt:");	// start the line

	for(x=0; x<3; x++)					// low line
		{
		Write_Display_Data(FCHR_L, 0);
		}

	Write_Display_Data(FCHR_R, 0);		// rise


	for(x=0; x<len; x++)				// high line
		{
		Write_Display_Data(FCHR_H, 0);
		}

	Write_Display_Data(FCHR_R, 0);		// fall

	for(x=0; x<((F_SYM_LEN - 2) - len); x++)					// low line
		{
		Write_Display_Data(FCHR_L, 0);
		}
	}


// Show selected filter
void display_filter(unsigned int filter)
	{

	draw_filter_graph((filter >> 7));	// pot / 128 to draw graphic

// dummy for now, just show the set value
//	sprintf(pbuf, "Flt: %.4X", filter); 			// Filter #
//	Write_String_LCD (FILTER_LINE, FILTER_COL, pbuf);
	}



// AF gain setup (HP volume setting)
// input = codec set value
void display_af_gain(int codec_val)
	{
	if(codec_val < 48)					// Mute ?
		sprintf(pbuf, "AF : Mute  "); 
	else
		sprintf(pbuf, "AF : %4ddB ", (codec_val-121));		// display value -73 --- +6
	Write_String_LCD (AF_GAIN_LINE, AF_GAIN_COL, pbuf);
	}


// LI gain setup (Line in amp)
// input = codec set value
void display_li_gain(int li_val)
	{
	float f;
	f = (float)li_val * 1.5;	// 1.5dB / bit
	f = f - 34.5;				// offset -34.5 dB

	sprintf(pbuf, "Lin:%5.1fdB ", (double)f);		// display value -34.5 --- +12
	Write_String_LCD (LI_GAIN_LINE, LI_GAIN_COL, pbuf);
	}

// RF attn + LI gain setup
// input = codec set value, RF attn value (select bits)
void display_rf_attn(int li_val, int rf)
	{
	float f;
	f = (float)li_val * 1.5;	// 1.5dB / bit
	f = f - 34.5;				// offset -34.5 dB

	f = f +(float)rf_attn_values[rf];	

	sprintf(pbuf, "Lin:%5.1fdB ", (double)f);		// display value -50.5 --- +12
	Write_String_LCD (LI_GAIN_LINE, LI_GAIN_COL, pbuf);
	}



// Display battery voltage
void display_batt_v(float v)
	{
	if(rit_on == 1)									// RIT on
		sprintf(pbuf, "Rit On   ");					// display Rit On message instead of Batt V
	else
		sprintf(pbuf, "Bat:%2.1fV", (double)v);		// display Battery voltage with one decimal

	Write_String_LCD (METER_LINE, METER_COL, pbuf);
	}


#if 0
// Display frequency with large fonts
// format is: 14.200.00

// test with normal dots
void display_freq_0(unsigned long freq)
	{
	Set_Large_XY_Addr(F_DISP_LINE, F_DISP_COL);	// set origin of the Frequency display

// start from 10_million digit
	if(freq/10000000 == 0)
		Write_Large_Char(' ', 0);				// leadign zero blanc for frequency display
	else
		Write_Large_Char((freq/10000000)+'0', 0);

	freq -= (freq/10000000)*10000000;

//million
	Write_Large_Char((freq/1000000)+'0', 0);
	freq -= (freq/1000000)*1000000;


	Write_Large_Char('.', 0);

//100_thousands
	Write_Large_Char((freq/100000)+'0', 0);
	freq -= (freq/100000)*100000;

//10_thousands
	Write_Large_Char((freq/10000)+'0', 0);
	freq -= (freq/10000)*10000;

//thousands
	Write_Large_Char((freq/1000)+'0', 0);
	freq -= (freq/1000)*1000;


	Write_Large_Char('.', 0);

//hundreds
	Write_Large_Char((freq/100)+'0', 0);
	freq -= (freq/100)*100;

//tens
	Write_Large_Char((freq/10)+'0', 0);
	freq -= (freq/10)*10;


	Write_String_LCD (F_DISP_LINE, 114,"Hz");
	}
#endif

// Display frequency with large fonts
// with small separator dots
// Digit blancking added for tuning resolution indication
void display_freq(unsigned long freq)
	{
	Set_Large_XY_Addr(F_DISP_LINE, F_DISP_COL);	// set origin of the Frequency display

// start from 10_million digit
	if(freq/10000000 == 0)
		Write_Large_Char(' ', 0);				// leadign zero blanc for frequency display
	else
		Write_Large_Char((freq/10000000)+'0', 0);

	freq -= (freq/10000000)*10000000;

//million
	Write_Large_Char((freq/1000000)+'0', 0);
	freq -= (freq/1000000)*1000000;

// first separator point
	Write_Char(',', 0);
	Set_Large_XY_Addr(F_DISP_LINE, (F_DISP_COL + 2*12 + 6) );	// continue column after small dp

//100_thousands
	Write_Large_Char((freq/100000)+'0', 0);
	freq -= (freq/100000)*100000;

if (eeprom.defval.step_idx == 2)
	{
	Write_Large_Char('_',0);
	Write_Large_Char('_',0);
	Write_Char('.', 0);
	Set_Large_XY_Addr(F_DISP_LINE, (F_DISP_COL + 5*12) + 2*6);	// continue column after small dp
	Write_Large_Char('_',0);
	Write_Large_Char('_',0);
	}
	else
		{	
		//10_thousands
		Write_Large_Char((freq/10000)+'0', 0);
		freq -= (freq/10000)*10000;

		//thousands
		Write_Large_Char((freq/1000)+'0', 0);
		freq -= (freq/1000)*1000;

		if (eeprom.defval.step_idx == 1)
			{
			Write_Char('.', 0);
			Set_Large_XY_Addr(F_DISP_LINE, (F_DISP_COL + 5*12) + 2*6);	// continue column after small dp
			Write_Large_Char('_',0);
			Write_Large_Char('_',0);
			}	
		else 
			{
			// second separator point
			Write_Char('.', 0);
			Set_Large_XY_Addr(F_DISP_LINE, (F_DISP_COL + 5*12) + 2*6);	// continue column after small dp

			//hundreds
			Write_Large_Char((freq/100)+'0', 0);
			freq -= (freq/100)*100;

			//tens
			Write_Large_Char((freq/10)+'0', 0);
			freq -= (freq/10)*10;
			}	
		}

	Write_String_LCD (F_DISP_LINE, 108,"kHz");
//	Write_String_LCD (F_DISP_LINE, 108,"MHz");
//	Write_String_LCD (F_DISP_LINE, 114,"Hz");
	}



// draw separator line, full screen left to right
// line = line#, dta = vertical pixel block, remember bottom = msb
void draw_separator_line(unsigned char line, unsigned char dta)
	{
	int x;

	Set_Display_XYAddr(line, 0);		// set starting point
	for(x=0; x< 128; x++)				// do whole line
		{
		Write_Display_Data(dta, 0);
		}
	}


// display run mode screen
// draw all run mode display objects
void display_run_screen(void)
	{
	Clear_Display(0);									// start with clear all
	draw_dummy_scope();									// draw dummy bandscope

	display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]);	// display freq, yet just test
	display_tune_step(eeprom.defval.step_idx);			// display new tuning step

	draw_separator_line(4, 0x04);						// upper pixels in a row

	display_mem_vfo();									// VFO & memory line
	display_mode(eeprom.defval.mode[eeprom.defval.mem], rx_tx);	// Mode & rx / tx state

	display_af_gain(hp_vol);							// volume pot
//	display_li_gain(li_vol);							// RF attn pot (Li gain)
	display_rf_attn(li_vol, rf_attn_bits);				// RF attenuator + Li gain

	display_filter(eeprom.defval.filter);				// filter pot / filter
	meas_lcd_disp_batt_volt();							// battery voltage
	}


// Message display system
// Two line essages are displayed in place of the frequency
//
void display_message(int msg_num)
	{
	Write_String_LCD (MSG_LINE,   MSG_COL,"---------------------");		// first clear the message space
	Write_String_LCD (MSG_LINE+1, MSG_COL,"---------------------");

	switch(msg_num)
		{
		default:
			Write_String_LCD (MSG_LINE,   MSG_COL,"Unknown Message      ");
			Write_String_LCD (MSG_LINE+1, MSG_COL,"?????????????????????");
		break;
	
		case MSG_CONF_SAVED:
			Write_String_LCD (MSG_LINE,   MSG_COL,"User Configuration   ");
			Write_String_LCD (MSG_LINE+1, MSG_COL,"saved to the EEPROM  ");
		break;

		case MSG_USER_CONFIG:
			Write_String_LCD (MSG_LINE,   MSG_COL,"                     ");
			Write_String_LCD (MSG_LINE+1, MSG_COL,"User Configuration   ");
		break;

		case MSG_1:
			Write_String_LCD (MSG_LINE,   MSG_COL,"Message 1            ");
			Write_String_LCD (MSG_LINE+1, MSG_COL,"+++++++++++++++++++++");
		break;

		case MSG_2:
			Write_String_LCD (MSG_LINE,   MSG_COL,"Message 2            ");
			Write_String_LCD (MSG_LINE+1, MSG_COL,"+++++++++++++++++++++");
		break;

		}
	}


// clear & redraw frequency display large line
void redraw_freq_info(void)
	{
	Write_String_LCD (F_DISP_LINE-1,  	MSG_COL,"                     ");							// first clear the freq disp space
	Write_String_LCD (F_DISP_LINE, 		MSG_COL,"                     ");

	display_freq(eeprom.defval.vfo[eeprom.defval.mem][eeprom.defval.vfo_sel[eeprom.defval.mem]]); 	// display freq
	display_mem_vfo();																				// display selected memory and the VFO
	display_tune_step(eeprom.defval.step_idx);														// display tuning step
	}
