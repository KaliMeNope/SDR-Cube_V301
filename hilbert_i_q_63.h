/*

 Cheap DSP dsPICradio board test Hilbert filter pair
 

 Juha Niinikoski OH2NLT 03.01.2009

*/

//*************************************************************************

// test Hilbert I Q filter pair coefficients

#define N_hilbert 63	

// Hilbert I/Q filter
// fs = 8000Hz


// Hilbert transformer coefficients
const int h_taps[N_hilbert] = {
     -73,      0,    -83,      0,   -128,      0,   -188,      0,   -266,
        0,   -366,      0,   -493,      0,   -654,      0,   -858,      0,
    -1121,      0,  -1471,      0,  -1958,      0,  -2692,      0,  -3962,
        0,  -6826,      0, -20818,      0,  20818,      0,   6826,      0,
     3962,      0,   2692,      0,   1958,      0,   1471,      0,   1121,
        0,    858,      0,    654,      0,    493,      0,    366,      0,
      266,      0,    188,      0,    128,      0,     83,      0,     73
};
