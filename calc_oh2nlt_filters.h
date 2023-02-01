//
// SDR_Cube DSP filter processing module
// OH2NLT / Juha Niinikoski 21.08.2010
//
// MPLAB v8.40, Microchip C30 v3.20
//

// All filters are calculated once at each sample in
// this assembler moduöe


#ifndef __OH2NLT_FILTERS_H
#define __OH2NLT_FILTERS_H

// defined in fir.s assembler module

// Note buffer lengths here words but in the assembler module they are bytes
//
//#define BL	256					// buffer  reservation(bytes)
#define BL	128					// buffer  reservation(words), must match with assembler file

//#define DLY_LINE 256			// general delay line
#define DLY_LINE 768			// general delay line

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
extern unsigned long firp_out2;	// post filter output SQUARED
extern unsigned int firp_out_r;	// post filter output RECTIFIED

// Delay line
extern unsigned int dly_line[DLY_LINE];		// General delay line
extern unsigned int dly_line_end;			// last address of ring buffer
extern unsigned int dly_line_in_ptr;		// delay line ring buffer insert pointer
extern unsigned int dly_line_out_ptr;		// delay line ring buffer output pointer		
extern int dly_line_in;						// delay line input
extern int dly_line_out;					// delay line output


// IIR filter data
// Microphone filter
extern int mic_iir_coef[];					// filter coefficients, 0,1,2 = A0,A1,A2  3,4 = B1,B2
extern volatile int mic_iir_dly[];			// filter delay line
extern volatile int mic_iir_in;				// Mic filter input
extern volatile int mic_iir_out;			// Mic filter output


// TX correction multipliers
extern unsigned int txbuf0;					// correction multiplier input
extern unsigned int txbuf1;

extern unsigned int txbuf0_cor;				// corrected output
extern unsigned int txbuf1_cor;

extern unsigned int txl_mul;				// gain correction multiplier, Left, for txbuf0
extern unsigned int txr_mul;				// gain correction multiplier, Right, for txbuf1
extern unsigned int txx_mul;				// phase correction multiplier, Left - Right mixing

// functions
extern void fir(void);						// calculate all filters, done for every sample

#endif
