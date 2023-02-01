/*

 Cheap DSP dsPICradio board IIR Microphone filter
 
 Juha Niinikoski OH2NLT 18.09.2005

*/

// selectable Mic filter 30.07.2013

//_____________________________________ #0 _
const int mic_coef_0[10] = {
// first IIR, direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0,		// -b1 = 0

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};

//_____________________________________ #1 _
const int mic_coef_1[10] = {
// first IIR, HPF 1500Hz, Ac 1.0

		15828,	// a0
		-31656,	// a1
		15828,	// a2
		18837,	// -b0, Note b0 and b1 sign swapped!
		-11707,	// -b1

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};

//_____________________________________ #2 _
const int mic_coef_2[10] = {
// first IIR, HPF 1700Hz, Ac 0.3

		16292,	// a0
		-32584,	// a1
		16292,	// a2
		22120,	// -b0, Note b0 and b1 sign swapped!
		-10279,	// -b1

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};

//_____________________________________ #3 _
const int mic_coef_3[10] = {
// first IIR, BPF 1900Hz, Ac 0.3

		14558,	// a0
		-29116,	// a1
		14558,	// a2
		16261,	// -b0, Note b0 and b1 sign swapped!
		-9204,	// -b1

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};


#if 0

// evaluation / measurement set 
// selectable Mic filter 26.04.2013

//_____________________________________ #0 _
const int mic_coef_0[10] = {
// first IIR, direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0,		// -b1 = 0

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};

//_____________________________________ #1 _
const int mic_coef_1[10] = {
// first IIR, HPF 1500Hz, Ac 0.8

		16193,	// a0
		-32386,	// a1
		16193,	// a2
		20525,	// -b0, Note b0 and b1 sign swapped!
		-11480,	// -b1

// second IIR direct
		32767,	// a0 = 1.0 (32767)
		0,		// a1 = 0
		0,		// a2 = 0
		0,		// -b0 = 0
		0		// -b1 = 0
};

//_____________________________________ #2 _
const int mic_coef_2[10] = {
// first IIR, HPF 2000Hz, Ac 0.3

		14632,	// a0
		-29265,	// a1
		14632,	// a2
		3454,	// -b0, Note b0 and b1 sign swapped!
		-22309,	// -b1

// second IIR HPF 2000Hz, Ac 0.3, second stage
		5814,
		-11628,
		5814,
		-15621,
		-6110
};

//_____________________________________ #3 _
const int mic_coef_3[10] = {
// first IIR, BPF 2000-2500Hz, Ac 0.3

		5861,	// a0
		0,		// a1
		-5861,	// a2
		-22880,	// -b0, Note b0 and b1 sign swapped!
		-24048,	// -b1

// second IIR
		9923,
		0,
		-9923,
		1328,
		-23359
};


#endif


//_____________________________________ test filters

#if 0

const int mic_coef[10] = {
// first IIR, HPF 2000Hz

		14632,	// a0
		-29265,	// a1
		14632,	// a2
		5454,	// -b0, Note b0 and b1 sign swapped!
		-22309,	// -b1

// second IIR LPF 2500Hz, Ac 0.8dB
		16193,
		32386,
		16193,
		-20525,
		-11480
};
#endif
