;
; Codec input data magnitude selector
;
; 
; Juha Niinikoski OH2NLT 25.11.2012
;

; added Codec input L | R abs value, rectified input signal

; processor definitions
.include "p33FJ256GP710.inc"

.section xcodecmag, bss, xmemory, align(2)

; Codec Magnitude selector variables

.global	_mag_sel				; Magnitude select. 0= direct, 1=mul by2, 2=mul by 4 etc
.global _mag_L_in				; Input D word L CH 
.global _mag_R_in				; Input D word R CH 
.global _mag_L_out				; scaled out L
.global _mag_R_out				; scaled out R

.global _codec_in_peak			; rectified codec input peak value before magnitude scaling

_mag_sel:		.space	2		; Max value 8
_mag_L_in:		.space	4		; LSB = lower address
_mag_R_in:		.space	4
_mag_L_out:		.space	2		; magnitude converted output values
_mag_R_out:		.space	2

_codec_in_peak:	.space	2		; rectified codec input peak value before magnitude scaling

;
; Magnitude Selector code
;
    .text                     	; Start of Code section

	.global	_codec_mag
_codec_mag:
; Save critical registers

        push    CORCON                          ; save context of CORCON
		push	ACCAL							; save DSP ACCU A
		push	ACCAH
		push	ACCAU
;
; Used, but not saved: W0, W1

;***************************************************
        mov     #0x00A0,w0                      ; Enable Accumulator A Normal Saturation, Fractional mode and and
                                                ; Data Space write Saturation
                                                ; as bits 5 and 7 are set in CORCON
        mov     w0, CORCON                      ; set PSV and saturation options

; Do Left Ch
		mov		_mag_L_in + 2, w0				; MSB part of the input
		lac		w0, A							; load ACCH and ACCU parts (set sign)

		mov		_mag_L_in, w0					; LSB part of the input
		mov		w0, ACCAL						; store directly to ACCA low part

		mov		_mag_sel, w0					; get magnitude select value
		neg		w0, w0							; make negative = shift left

		sftac	A, w0							; select magnitude

		sac		A, w0
		mov		w0, _mag_L_out					; store result

; Do Right Ch
		mov		_mag_R_in + 2, w0				; MSB part of the input
		lac		w0, A							; load ACCH and ACCU parts (set sign)

		mov		_mag_R_in, w0					; LSB part of the input
		mov		w0, ACCAL						; store directly to ACCA low part

		mov		_mag_sel, w0					; get magnitude select value
		neg		w0, w0							; make negative, shifr left

		sftac	A, w0							; select magnitude

		sac		A, w0
		mov		w0, _mag_R_out					; store result

; codec input peak value
		mov		_mag_L_in + 2, w0				; MSB part of the L input
		btsc 	w0, #0xF						; test msb (negative number)
		neg 	w0, w0							; take 2's complement, rectify
;		mov 	w0, _codec_in_peak				; store abs value

		mov		_mag_R_in + 2, w1				; MSB part of the R input
		btsc 	w1, #0xF						; test msb (negative number)
		neg 	w1, w1							; take 2's complement, rectify

		cpslt	w1, w0							; do nothing if L was greater
		mov		w1, w0 							; if R greater replace with R value	

		mov 	w0, _codec_in_peak				; store abs value

;***************************************************
;
; Restore critical registers
;
		pop 	ACCAU							; restore DSP ACCU A
		pop		ACCAH
		pop		ACCAL
        pop     CORCON                          ; restore context of CORCON
        nop
;
		return
;
