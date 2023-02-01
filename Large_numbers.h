//
// OH2NLT /  large numbers character generator
//
// 7x12 BCD numbers 21.12.2011
// 9x14 BCD numbers added 21.12.2011



#ifndef __CHGEN_LNUM_H
#define __CHGEN_LNUM_H

const unsigned char Lnum7x12_L1[11][7]=
{
// Special 7x12 numbers font, line 1
//
{0xF0,0xF8,0x1C,0x0C,0x1C,0xF8,0xF0}, // 0x00
{0x10,0x18,0xFC,0xFC,0x00,0x00,0x00}, // 0x01
{0x30,0x38,0x1C,0x8C,0xDC,0x7C,0x38}, // 0x02
{0x30,0x38,0x1C,0x8C,0x9C,0xF8,0x70}, // 0x03
{0xC0,0xE0,0x30,0x18,0xFC,0xFC,0x00}, // 0x04
{0xFC,0xFC,0x8C,0x8C,0x8C,0x8C,0x8C}, // 0x05
{0xF0,0xF8,0x8C,0x84,0x8C,0x18,0x10}, // 0x06
{0x0C,0x0C,0x0C,0x0C,0x8C,0xFC,0x7C}, // 0x07
{0x70,0xF8,0x9C,0x8C,0x9C,0xF8,0x70}, // 0x08
{0x70,0xF8,0x8C,0x04,0x8C,0xF8,0xF0}, // 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // 0x0A = blanco
};


const unsigned char Lnum7x12_L2[11][7]=
{
// Special 7x12 numbers font, line 1
//
{0x0F,0x1F,0x38,0x30,0x38,0x1F,0x0F}, // 0x00
{0x30,0x30,0x3F,0x3F,0x30,0x30,0x00}, // 0x01
{0x3C,0x3E,0x33,0x31,0x30,0x30,0x30}, // 0x02
{0x0C,0x1C,0x38,0x31,0x39,0x1F,0x0E}, // 0x03
{0x03,0x03,0x03,0x03,0x3F,0x3F,0x03}, // 0x04
{0x09,0x19,0x31,0x21,0x31,0x1F,0x0F}, // 0x05
{0x0F,0x1F,0x31,0x20,0x31,0x1F,0x0E}, // 0x06
{0x38,0x3C,0x06,0x03,0x01,0x00,0x00}, // 0x07
{0x0E,0x1F,0x39,0x31,0x39,0x1F,0x0E}, // 0x08
{0x08,0x18,0x31,0x21,0x31,0x1F,0x0F}, // 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // 0x0A = blanco
};


const unsigned char Lnum9x14_L1[11][9]=
{
// Special 9x14 numbers font, line 1
//
{0xF0,0xFC,0x0E,0x06,0x06,0x06,0x0E,0xFC,0xF0}, // 0x00, open zero
//{0xF0,0xFC,0x0E,0x06,0x86,0xC6,0x6E,0xFC,0xF0}, // 0x00, zero with diagonal bar 
{0x10,0x18,0x1C,0xFE,0xFE,0x00,0x00,0x00,0x00}, // 0x01
{0x18,0x1C,0x0E,0x06,0x06,0x86,0xCE,0x7C,0x38}, // 0x02
{0x18,0x1C,0x0E,0x86,0x86,0x86,0xCE,0x7C,0x38}, // 0x03
{0x00,0x00,0xC0,0xF0,0x38,0x0E,0xFE,0xFE,0x00}, // 0x04
{0xFE,0xFE,0xC6,0xC6,0xC6,0xC6,0xC6,0x86,0x06}, // 0x05
{0xF8,0xFC,0x8E,0x86,0x86,0x86,0x8E,0x1C,0x18}, // 0x06
{0x06,0x06,0x06,0x06,0x86,0xC6,0x66,0x3E,0x1E}, // 0x07
{0x38,0x7C,0xCE,0x86,0x86,0x86,0xCE,0x7C,0x38}, // 0x08
{0x78,0xFC,0xCE,0x86,0x86,0x86,0xCE,0xFC,0xF8},	// 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // 0x0A = blanco
};


const unsigned char Lnum9x14_L2[11][9]=
{
// Special 9x14 numbers font, line 2
//
{0x0F,0x3F,0x70,0x60,0x60,0x60,0x70,0x3F,0x0F}, // 0x00, open zero
//{0x0F,0x3F,0x76,0x63,0x61,0x60,0x70,0x3F,0x0F}, // 0x00, zero with diagonal bar 
{0x60,0x60,0x60,0x7F,0x7F,0x60,0x60,0x60,0x00}, // 0x01
{0x70,0x78,0x6C,0x66,0x63,0x61,0x60,0x60,0x60}, // 0x02
{0x18,0x38,0x70,0x61,0x61,0x61,0x73,0x3E,0x1C}, // 0x03
{0x0E,0x0F,0x0F,0x0C,0x0C,0x0C,0x7F,0x7F,0x0C}, // 0x04
{0x18,0x38,0x70,0x60,0x60,0x60,0x70,0x3F,0x1F}, // 0x05
{0x1F,0x3F,0x73,0x61,0x61,0x61,0x73,0x3F,0x1E}, // 0x06
{0x00,0x7C,0x7E,0x03,0x01,0x00,0x00,0x00,0x00}, // 0x07
{0x1C,0x3E,0x73,0x61,0x61,0x61,0x73,0x3E,0x1C}, // 0x08
{0x18,0x38,0x61,0x61,0x61,0x61,0x61,0x3F,0x1F}, // 0x09
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // 0x0A = blanco
};

#endif