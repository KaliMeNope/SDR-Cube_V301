/*

SDR Cube band pass filter coefficients
OH2NLT / Juha Niinikoski 27.08.2010

N 101
fs 8000
f1 10
f2 200
f3 700
f4 890

Data is Decimal : 1.0 =>      32768
*/

const int bpf_200_700_taps[101] = {
         -30,
          -5,
          12,
          42,
          80,
         115,
         135,
         131,
         100,
          48,
          -8,
         -50,
         -59,
         -30,
          29,
          99,
         149,
         151,
          91,
         -22,
        -160,
        -281,
        -343,
        -322,
        -225,
         -89,
          25,
          57,
         -31,
        -240,
        -520,
        -787,
        -951,
        -946,
        -760,
        -452,
        -136,
          48,
         -11,
        -359,
        -936,
       -1584,
       -2079,
       -2192,
       -1763,
        -755,
         713,
        2390,
        3941,
        5035,
        5429,
        5035,
        3941,
        2390,
         713,
        -755,
       -1763,
       -2192,
       -2079,
       -1584,
        -936,
        -359,
         -11,
          48,
        -136,
        -452,
        -760,
        -946,
        -951,
        -787,
        -520,
        -240,
         -31,
          57,
          25,
         -89,
        -225,
        -322,
        -343,
        -281,
        -160,
         -22,
          91,
         151,
         149,
          99,
          29,
         -30,
         -59,
         -50,
          -8,
          48,
         100,
         131,
         135,
         115,
          80,
          42,
          12,
          -5,
         -30
};
