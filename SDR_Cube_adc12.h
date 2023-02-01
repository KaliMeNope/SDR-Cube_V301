//
// 33FJ256GP710 ADC12 routines
// OH2NLT / Juha Niinikoski 18.08.2009
//

// module function definitions

#ifndef __ADC12_H
#define __ADC12_H

void init_adc12(void);							// init ADC12 for SDR Cube use
int convert_adc12(unsigned int);				// convert selected ADC channel

int get_scaled_batt(unsigned int);				// scale battery voltage
void disp_all_adc_ch(void);						// test printout
void disp_batt_volt(void);						// measure & print battery voltage

void meas_lcd_disp_batt_volt(void);				// test, scale with floats

#endif
