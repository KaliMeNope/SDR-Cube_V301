
/**	@file timer.c
 * 	@brief Timer handling routines for SDR Cube flasher
 * 	@version 0.02, 18.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 * 	This module contains the timer handling routines
 * 	for the dsPIC flasher.
 */

#include "cubeflash.h"
#include "cubepic.h"


/**	Timeout check
 *
 *	@return @a true if the timer has run out.
 */

bool timeout(void)
	{
	return (IFS0bits.T3IF) ? true : false;
	}


/**	Timer initialization
 *
 *	@param seconds Timeout value
 *
 *	The function initializes the timer and
 *	starts a timeout of @a seconds. If the
 *	@a seconds value is 0xff, the timer will
 *	not be started, creating effectively a
 *	never-ending timeout.
 */

void tmr_init(uint8_t seconds)
	{
	uint32_t count;

	/* set up timer */

	IFS0bits.T3IF = 0;          /* clear timer3 interrupt flag */
	IEC0bits.T3IE = 0;          /* disable timer interrupts */

	if (seconds != 0xFF)
		{
		T2CONbits.T32 = 1;      /* increment at every master clock */
		count = FCY * seconds;

		PR3 = count >> 16;
		PR2 = count;

		T2CONbits.TON = 1;      /* start timing */
		}
	}

