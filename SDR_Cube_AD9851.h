/*

 AD9850 DDS chip driver

 OH2NLT / Juha Niinikoski 02.04.2004

 AD9851 test code added 26.08.2004

*/


#ifndef __AD9850_H
#define __AD9850_H


void dds_load(unsigned long);				// load AD9850, argument = DDS phase adder 	
void dds_init(void);						// Init AD9850 interface

void set_dds_clk(unsigned long);			// set DDS clock frequency & calculate internal multipliers
unsigned long calc_dds_n(unsigned long);	// calculate DDS N
void set_dds_freq(unsigned long);			// set DDS output frequency, calculate & set N

#endif
