;
; Simple FIR filter(s) module
; Filters are calculated at each new sample
;
; Juha Niinikoski OH2NLT 27.05.2005
; 
; adapted for Cheap DSP board
; Microphone IIR filter added 18.09.2005

; adapted to SDR Cube project 10.08.2010
; general audio delay line added 24.08.2010
; BUG corrected, missing push DSP ACCU instructions added 15.01.2011
; TX gain multipliers added 15.01.2011
; CORCON ACCSAT mode corrected 24.12.2012
; RX I  and Q correction multiplier bug corrected 26.12.2012
; Bug corrected in the Mic IIR filter. TNX to OH2UG for finding the bug. 20.04.2013
; More bugs corrected in the Mic IIR filter & FIR filters -> improved responce 26.04.2013

; processor definitions
.include "p33FJ256GP710.inc"

;
; Y-data, be carefull with alignment, counted at bytes
; Modulo addressing needs zero boundary addresses
;
; Hitec style
;	psect	yfir,class=YDATA,space=1,reloc=256,pad=256

; C30 style
	.section yfir, bss, ymemory, align(256)

_firl_taps:		.space	256		; Filter coeffisients (bytes)
_firr_taps:		.space	256		; Filter coeffisients (bytes)
_firp_taps:		.space	256		; Filter coeffisients (bytes)


; Left CH

	.global	_firl_taps		; tap vector
	.global	_firl_taps_end	; last address of tap vector
	.global	_firl_n			; filter length (N)

_firl_taps_end:	.space	2		; Last tap
_firl_n:		.space	2		; Filter length

; Right CH

	.global	_firr_taps		; tap vector
	.global	_firr_taps_end	; last address of tap vector
	.global	_firr_n			; filter length (N)

_firr_taps_end:	.space	2		; Last tap
_firr_n:		.space	2		; Filter length

; Post filter

	.global	_firp_taps		; tap vector
	.global	_firp_taps_end	; last address of tap vector
	.global	_firp_n			; filter length (N)

_firp_taps_end:	.space	2		; Last tap
_firp_n:		.space	2		; Filter length


; MIC IIR filter

	.global	_mic_iir_coef	; IIR Multipliers
_mic_iir_coef:	.space	20		; Filter coefficients (bytes)


;
; X-data, be carefull with alignment
; Modulo addressing needs zero boundary addresses for delay lines
;
; Delay lines, watch for link address
; Hitec style
;	psect	xfir,class=XDATA,space=1,reloc=256,pad=256
; C30 stylw
	.section xfir, bss, xmemory, align(256)

	.global	_firl_dly		; filter delay line
	.global	_firr_dly		; filter delay line
	.global	_firp_dly		; podt filter delay line

_firl_dly:		.space	256		; filter delay line (bytes)
_firr_dly:		.space	256		; filter delay line (bytes)
_firp_dly:		.space	256		; filter delay line (bytes)

_dly_line:			.space	1536		; general delay line (bytes) 3 * 256 samples
;_dly_line:			.space	512		; general delay line (bytes)

_dly_line_end:		.space	2		; last address of delay line
_dly_line_in_ptr:	.space	2		; delay buffer input pointer
_dly_line_out_ptr:	.space	2		; delay buffer output pointer
_dly_line_in:		.space	2		; delay buffer input data
_dly_line_out:		.space	2		; delay buffer output data

; General delay line

	.global		_dly_line			; Generaldelay line
	.global		_dly_line_end		; last address of delay line
	.global		_dly_line_in_ptr 	; insert pointer to delay line
	.global		_dly_line_out_ptr	; output pointer to delay line
	.global		_dly_line_in		; in sample
	.global		_dly_line_out		; out sample


; Mic IIR filter variables
	.global	_mic_iir_dly	; IIR Multipliers
	.global	_mic_iir_in		; mic filter input
	.global	_mic_iir_out	; mic filter output
			
_mic_iir_dly:	.space	20		; delay line (bytes)
_mic_iir_in:	.space	2		; post filter filter input
_mic_iir_out:	.space	2		; post filter filter output

; Left CH variables

	.global	_firl_dly_end	; last address of delay line
	.global	_firl_dly_ptr	; insert pointer to delay line
	.global	_firl_in		; Left input
	.global	_firl_mul		; Left multiplier
	.global	_firl_out		; Left output	
	.global	_firl_mix		; Left mixer
	.global _firl_mix_out	; Left multiplier out
	.global _x_mul			; Cross multiplier
	.global _fir_gain		; L and R filter gain, range 0 - 32767 ( 0 - 1.0 )	

_firl_dly_end:	.space	2		; last address of delay line
_firl_dly_ptr:	.space	2		; delay buffer input pointer
_firl_in:		.space	2		; Left filter input
_firl_mul:		.space	2		; Left input multiplier
_firl_out:		.space	2		; Left filter output
_firl_mix:		.space	2		; Left output multiplier(mixer)
_firl_mix_out:	.space	2		; Left multiplied output
_x_mul:			.space	2		; Cross multiplier
_x_mul_result:	.space	2		; RX Cross multiplier result
_fir_gain:		.space	2		; L and R filter gain, range 0 - 32767 ( 0 - 1.0 )

;Right CH variables

	.global	_firr_dly_end	; last address of delay line
	.global	_firr_dly_ptr	; insert pointer to delay line
	.global	_firr_in		; Right input
	.global	_firr_mul		; Right multiplier
	.global	_firr_out		; Right output
	.global	_firr_mix		; Right mixer
	.global _firr_mix_out	; Right multiplier out
	
_firr_dly_end:	.space	2		; last address of delay line
_firr_dly_ptr:	.space	2		; delay buffer input pointer
_firr_in:		.space	2		; Right filter input
_firr_mul:		.space	2		; Right input multiplier
_firr_out:		.space	2		; Right filter output
_firr_mix:		.space	2		; Right output multiplier(mixer)
_firr_mix_out:	.space	2		; Right multiplied output

;post filter variables

	.global	_firp_dly_end	; last address of delay line
	.global	_firp_dly_ptr	; insert pointer to delay line
	.global	_firp_in		; post filter input
	.global	_firp_out		; post filter output
	.global	_firp_out2		; post filter output squared (unsigned int)
	.global	_firp_out_r		; rectified post filter output
	
_firp_dly_end:	.space	2		; last address of delay line
_firp_dly_ptr:	.space	2		; delay buffer input pointer
_firp_in:		.space	2		; post filter filter input
_firp_out:		.space	2		; post filter filter output
_firp_out2:		.space	4		; post filter filter output squared, lower address = LSB
_firp_out_r:	.space	2		; rectified post filter filter output



; TX correction variables &  multipliers

	.global	_txbuf0				; TX buffer0, correction input
	.global	_txbuf1				; TX buffer1, correction input
	.global	_txbuf0_cor			; TX buffer0, corrected out
	.global	_txbuf1_cor			; TX buffer1, corrected out

	.global _txl_mul			; TX Left correction multiplier
	.global _txr_mul			; TX Right correction multiplier
	.global _txx_mul			; TX Phaxe correction multiplier, L/R mixing

_txbuf0:		.space	2		; TX buffer0, correction input
_txbuf1:		.space	2		; TX buffer1, correction input
_txbuf0_cor:	.space	2		; TX buffer0, corrected out
_txbuf1_cor:	.space	2		; TX buffer1, corrected out
_tx_mul_result:	.space	2		; TX Cross multiplier resul

_txl_mul:		.space	2		; TX Left correction multiplier
_txr_mul:		.space	2		; TX Right correction multiplier
_txx_mul:		.space	2		; TX Phaxe correction multiplier, L/R mixing

;
; Filter code
;
;	psect	text,class=CODE,delta=2,reloc=4
    .text                     ; Start of Code section

	.global	_fir

;
; Tap vectors in Y-memory
; Delay (ring)buffer in X-memory
;

_fir:
; Save critical registers

        push    MODCON                          ; save context of MODCON
        push    XMODSRT                         ; save context of YMODSRT
        push    XMODEND                         ; save context of YMODEND
        push    CORCON                          ; save context of CORCON
        push    PSVPAG                          ; save context of PSVPAG
		push	RCOUNT							; save context of RCOUNT
		push	SR								; save context of SR
        push    w5                              ; save context of w5
        push    w6                              ; save context of w6
        push    w8                              ; save context of w8
        push    w10                             ; save context of w10

		push	ACCAL							; save DSP ACCU A
		push	ACCAH
		push	ACCAU
;
; Used, but not saved: W0, W1
;
; Set core mode for DSP calculations
;
;        mov     #0x00B0,w8                     ; Enable Accumulator A Super Saturation and
        mov     #0x00A0,w8                      ; Enable Accumulator A Normal Saturation, Fractional mode and
                                                ; Data Space write Saturation
                                                ; as bits 5 and 7 are set in CORCON
        mov     w8, CORCON                      ; set PSV and saturation options		
;
; Set modulo addressing & delay buffer address
; Note W10 or W11 are only valid Y-memory modulo pointers
;
        mov     #0x8FF8, w8						; set XMD = 8 and YMD = no
        mov     w8, MODCON						; enable Y Modulus, next instruction
												; can not be indirect addressed 
;
; Multiply input operands before filter
; Can be used for mixing or gain control
;
		mov		_firl_in, w5					; get new Left CH sample
		mov		_firl_mul, w6					; Left CH IQ correction multiplier
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w0							; result for next operation
;
; Cros multiplier, Left ch _x_mul
		mov		w0, w5							; get just scaled Left data
		mov		_x_mul, w6						; get cross multiplier		
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w1							; round & transfer to register
		mov		w1, _x_mul_result				; save result ( high part )
;
; Filter gain (AGC gain)
		mov		w0, w5
		mov		_fir_gain, w6					; get IQ filter gain
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   A,w0							; result for filter input

;		mov		_firl_in, w0					; get new Left CH sample, no multiply
;
; Calculate Left Channell filter
;	
        mov     #_firl_dly,w8					; get ring buffer location
        mov     w8, XMODSRT                    ; YMODSRT = base address of delay line

		mov		_firl_dly_end, w8				; load last member address
        mov     w8, XMODEND                    ; YMODEND = end address of delay line

		mov		_firl_n, w1						; Number of taps
		dec		w1, w1							; W1 = Number of taps - 1
 
;
; Insert new sample to ring buffer @ X-memory space
; Modulo addressing works only in X-memory with non MAC class instructions
;
		mov _firl_dly_ptr, w8					; get delay line pointer
		mov w0, [w8++]							; store new sample (w0)
		mov	w8, _firl_dly_ptr					; w10 point oldest sample
;
; Tap vector @ Y-memory
;
		mov _firl_taps_end, w10					; tap vector end to w10
;
; Calculate Filter
; W10 point to tap vector last member @ Y-memory
; tap vector is read backwards
; W8 point to oldest sample at delay line @ X-memory
; sample buffer is read forward direction
;
		clr    A, [w8]+=2, w5, [w10]-=2, w6		; clear ACCA & get last tap & oldest sample
		repeat w1								; repeat count (W1) = Number of taps - 1
		mac w5*w6, A,  [w8]+=2, w5,  [w10]-=2, w6
;
; round and store result in AccA to output register
;
 		sac.r   a,w5
;
; Left out mixer
;
		mov		_firl_mix, w6					; mixing signal
		mpy		w5*w6, A						; multiply without prefetch
       	sac.r   a,w0
;
		mov		w0, _firl_mix_out;				; Left multiplier out
		mov		w5, _firl_out;					; Left output
;
;***************************************************
;
; Multiply input operands before filter
; Can be used for mixing or gain control
;
		mov		_firr_in, w5					; get new Left CH sample
		mov		_firr_mul, w6					; Right CH IQ correction multiplier
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w0							; result for next operation

; Add cross multiplier
		mov _x_mul_result, w1					; get X multiplier result
		add w1, w0, w0							; W0 is now corrected with cross multiplier

; Filter gain (AGC gain)
		mov		w0, w5
		mov		_fir_gain, w6					; get IQ filter gain
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   A,w0							; result for filter input

;		mov		_firr_in, w0					; get new Right CH sample
;
; Right Channell filter
;	
        mov     #_firr_dly,w8					; get ring buffer location
        mov     w8, XMODSRT                    ; YMODSRT = base address of delay line

		mov		_firr_dly_end, w8				; load last member address
        mov     w8, XMODEND                    ; YMODEND = end address of delay line

		mov		_firr_n, w1						; Number of taps
		dec		w1, w1							; W1 = Number of taps - 1 
;
; Insert new sample to ring buffer @ X-memory space
; Modulo addressing works only in X-memory with non MAC class instructions
;
		mov _firr_dly_ptr, w8					; get delay line pointer
		mov w0, [w8++]							; store new sample (w0)
		mov	w8, _firr_dly_ptr					; w10 point oldest sample
;
; Tap vector @ Y-memory
;
		mov _firr_taps_end, w10					; tap vector end to w10
;
; Calculate Filter
; W10 point to tap vector last member @ Y-memory
; tap vector is read backwards
; W8 point to oldest sample at delay line @ X-memory
; sample buffer is read forward direction
;
		clr    A, [w8]+=2, w5, [w10]-=2, w6			; clear ACCA & get last tap & oldest sample
		repeat w1								; repeat count (W1) = Number of taps - 1
		mac w5*w6, A,  [w8]+=2, w5,  [w10]-=2, w6
;
; round and store result in AccA to output register
;
 		sac.r   a,w5
;
; Right out mixer
;
		mov		_firr_mix, w6					; mixing signal
		mpy		w5*w6, A						; multiply without prefetch
       	sac.r   a,w0
;
		mov		w0, _firr_mix_out;				; Right multiplier out
		mov		w5, _firr_out;					; Right output
;	
;
;***************************************************
;
; Post filter

		mov		_firp_in, w0					; get input
;
        mov     #_firp_dly,w8					; get ring buffer location
        mov     w8, XMODSRT                    ; YMODSRT = base address of delay line

		mov		_firp_dly_end, w8				; load last member address
        mov     w8, XMODEND                    ; YMODEND = end address of delay line

		mov		_firp_n, w1						; Number of taps
		dec		w1, w1							; W1 = Number of taps - 1 
;
		mov _firp_dly_ptr, w8					; get delay line pointer
		mov w0, [w8++]							; store new sample (w0)
		mov	w8, _firp_dly_ptr					; w10 point oldest sample
;
		mov _firp_taps_end, w10					; tap vector end to w10
;
		clr    A, [w8]+=2, w5, [w10]-=2, w6		; clear ACCA & get last tap & oldest sample

		repeat w1								; repeat count (W1) = Number of taps - 1
		mac w5*w6, A,  [w8]+=2, w5,  [w10]-=2, w6
;
 		sac.r   a,w0
		mov		w0, _firp_out					; post filter output
		mov		w0, w5;							; take a copy
;
		mul.ss	w0, w0, w0						; square result to w0, w1, w0 = LSD
		mov w0, _firp_out2						; LSB
		mov w1, _firp_out2 + 2					; MSB
;
		btsc w5, #0xF							; test msb (negative number)
		neg w5, w5								; take 2's complement
		mov w5, _firp_out_r						; store abs value

;***************************************************
;
; General Delay line 23.08.2010
; Modulo addressing must be on when entering this section
;
		mov		_dly_line_in, w0				; get input

        mov     #_dly_line,w8					; get ring buffer start location
        mov     w8, XMODSRT                    	; XMODSRT = base address of delay line

		mov		_dly_line_end, w8				; load last member address
        mov     w8, XMODEND                    	; XMODEND = end address of delay line

		mov 	_dly_line_in_ptr, w8			; get delay line in pointer
		mov 	w0, [w8++]						; store new sample (w0)
		mov		w8, _dly_line_in_ptr			; w8 point now oldest sample

		mov 	_dly_line_out_ptr, w8			; delay line out pointer
		mov		[w8++], w0						; get sample
		mov		w8, _dly_line_out_ptr			; save updated pointer for next round

		mov		w0, _dly_line_out				; store output 

;	
;
;***************************************************
;
; Mic IIR filter
;
; delay line & coefficient placement
;
; IIR_1 filter
; [0] = a0, [1] = a1, [2] = a2,
; [3] = b1, [4] = b2
;
; IIR_2 filter
; [5] = a0, [6] = a1, [7] = a2,
; [8] = b1, [9] = b2
; Note b1, b2 coefficients sign must be swapped !
; 
; 0. set correct addressing mode
; 1. set new sample to delay line[0]
; 2. MAC delay line with coefficients
; 3. shift delay line one right
; 4. set new output to delay line[3]
;
        clr     MODCON                          ; disable modulo addressing

; First IIR section
		mov		_mic_iir_in, w8					; get input	
		mov		w8, _mic_iir_dly				; move sample to IIR input

		mov		#_mic_iir_dly, w8				; set delay line pointer w8
		mov		#_mic_iir_coef, w10				; set coefficient pointer to w10
		clr		A, [w8]+=2, w5, [w10]+=2, w6	; clear ACCA & get first coef & first sample, also post increment pointers
		repeat #4								; repeat Number of coef -1
		mac		w5*w6, A,  [w8]+=2, w5,  [w10]+=2, w6
		sac.r   a,w0							; result to w0

		mov		w0, _mic_iir_dly+10				; store result to next section input

		mov		#_mic_iir_dly+8, w8				; prepare delay lins shift. W8 = to ptr
		mov		#_mic_iir_dly+6, w10			; W10 = from ptr
		repeat #3								; delay line len -1-1
		mov		[w10--], [w8--]					; shift delayline right
		mov		w0, _mic_iir_dly+6				; set new sample to delay line for next round

; Second IIR section
		mov		#_mic_iir_dly+10, w8			; set delay line pointer w8
		mov		#_mic_iir_coef+10, w10			; set coefficient pointer to w10
		clr		A, [w8]+=2, w5, [w10]+=2, w6	; clear ACCA & get first coef & first sample, also post increment pointers
		repeat #4								; repeat Number of coef -1-1
		mac		w5*w6, A,  [w8]+=2, w5,  [w10]+=2, w6
 		sac.r   a,w0							; result to w0

		mov		w0, _mic_iir_out				; store result

		mov		#_mic_iir_dly+18, w8			; prepare delay lins shift. W8 = to ptr
		mov		#_mic_iir_dly+16, w10			; W10 = from ptr
		repeat #3								; delay line len -1-1
		mov		[w10--], [w8--]					; shift delayline right
		mov		w0, _mic_iir_dly+16				; set new sample to delay line for next round
test:
;***************************************************
;
; TX gain & phase correction
; Inputs txbuf0, txbuf1
; Outputs txbuf0_cor, txbuf1_cor
; Multipliers txl_mul, txr_mul, txx_mul
;

; Left, buf0
		mov		_txbuf0, w5						; get new Left CH sample
		mov		_txl_mul, w6					; get Left CH multiplier
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w0							; round
		mov		w0,_txbuf0_cor					; save result

; Cros multiplier, Left ch _x_mul
		mov		w0, w5							; get just scaled Left data
		mov		_txx_mul, w6						; get cross multiplier		
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w1							; round & transfer to register W1
;		mov		w1, _tx_mul_result				; save result ( high part )

; Right, buf1
		mov		_txbuf1, w5						; get new Right CH sample
		mov		_txr_mul, w6					; get right CH multiplier
		mpy		w5*w6, A						; multiply without prefetch
		sac.r   a,w0							; round
		mov		w0,_txbuf1_cor					; save result

; add cross multiplier
;		mov 	_tx_mul_result, w1				; get X multiplier result
		add 	w1, w0, w0						; W0 is now corrected with cross multiplier
		mov		w0,_txbuf1_cor					; save result

;	
;***************************************************
;
; Restore critical registers
;
exit_filters:

		pop 	ACCAU							; restore DSP ACCU A
		pop		ACCAH
		pop		ACCAL

        pop     w10                             ; restore context of w10
        pop     w8                              ; restore context of w8
        pop     w6                              ; restore context of w6
        pop     w5                              ; restore context of w5
		pop		SR								; restore context of SR
		pop		RCOUNT							; restore context of RCOUNT
        pop     PSVPAG                          ; restore context of PSVPAG
        pop     CORCON                          ; restore context of CORCON
        pop     XMODEND                         ; restore context of YMODEND
        pop     XMODSRT                         ; restore context of YMODSRT
        pop     MODCON                          ; restore context of MODCON
        nop
;
		return
;
