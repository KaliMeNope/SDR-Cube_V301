/**	@file serio.c
 * 	@brief Serial I/O routines for flasher
 * 	@version 0.02, 18.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 * 	This module contains the serial I/O routines
 * 	for the dsPIC flasher.
 */

#include "cubeflash.h"
#include "cubepic.h"


/*   Module data   */

#define BRGVAL ((FCY / BRATE) / 16 - 1) /**< Bit rate divisor */

#define S_FERR (1 << 2)         /**< Status: framing error */
#define S_OERR (1 << 1)         /**< Status: overrun error */
#define S_RXDA (1 << 0)         /**< Status: data available */


/**	Poll receiver ready
 *
 *	@return @a true if a received byte is available
 *
 *	The function polls the serial line receiver, checks
 *	the line status and returns true if there is a byte
 *	without error conditions in the receiver buffer.
 */

bool rxrdy(void)
	{
	uint16_t sts;

	sts = U1STA;

	if (sts & S_OERR)           /* overrun? */
		{
		U1STAbits.OERR = 0;
		return false;
		}

	if (sts & S_FERR)           /* framing error? */
		return false;

	if (sts & S_RXDA)           /* data available? */
		return true;

	return false;
	}


/**	Receive a byte from serial line
 *
 *	@return Byte from serial line
 *
 *	The function receives a byte from the serial
 *	line, waiting till a byte is available.
 */

uint8_t getb(void)
	{
	do
		;
	while (!rxrdy());

	return U1RXREG;
	}


/**	Send a byte to serial line
 *
 *	@param b Byte to send
 *
 *	The function sends the byte @a b to the serial line.
 */

void putb(uint8_t b)
	{
	do
		;
	while (!U1STAbits.TRMT);

	U1TXREG = b;
	}


/**	Serial line initialization
 *
 *	The function initializes the serial line handler.
 */

void sio_init(void)
	{
	U1BRG  = BRGVAL;    /* set up bit rate */
	U1MODE = 0x8000; 	/* 8-n-1, alt pins, and enable */
	U1STA  = 0x0400; 	/* reset status, enable transmit */
	}
