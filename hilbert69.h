/*

 Cheap DSP dsPICradio board test Hilbert filter pair
 Calculated with own Excel application

 Juha Niinikoski OH2NLT 12.09.2005

*/

//*************************************************************************

// test Hilbert filter pair coefficients

#define N 69	

// Hilbert filter pair
// fs = 8000Hz
// fo = 1400 Hz

// sin filter
const int l_taps[N] = {
1,
2,
0,
9,
8,
2,
33,
10,
19,
75,
-1,
72,
117,
-25,
183,
115,
-43,
344,
-1,
-1,
490,
-285,
169,
465,
-755,
509,
-1,
-1383,
994,
-1475,
-2250,
1519,
-7192,
-5936,
17375,
18268,
1139,
2091,
4415,
0,
1952,
1903,
-1,
1567,
754,
151,
1071,
206,
249,
606,
0,
250,
267,
-38,
183,
76,
-19,
100,
0,
0,
38,
-15,
5,
8,
-9,
3,
0,
-2,
0
};

// cos filter
const int r_taps[N] = {
0,
-2,
0,
3,
-9,
8,
5,
-15,
38,
0,
0,
100,
-19,
76,
183,
-38,
267,
250,
0,
606,
249,
206,
1071,
151,
754,
1567,
-1,
1903,
1952,
0,
4415,
2091,
1139,
18268,
17375,
-5936,
-7192,
1519,
-2250,
-1475,
994,
-1383,
-1,
509,
-755,
465,
169,
-285,
490,
0,
-1,
344,
-43,
115,
183,
-25,
117,
72,
-1,
75,
19,
10,
33,
2,
8,
9,
0,
2,
1
};
