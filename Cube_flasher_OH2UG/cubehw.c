/**	@file cubehw.c
 * 	@brief SDR Cube hardware-dependent routines for flasher
 * 	@version 0.03, 21.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 * 	This module contains the flash loader functions which
 * 	are directly depending on SDR Cube hardware.
 */

#include "cubeflash.h"


/* SDR Cube processor configuration words */

	_FBS		( RBS_NO_RAM & BSS_NO_FLASH & BWRP_WRPROTECT_OFF )
	_FSS		( RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF )
	_FGS		( GSS_OFF & GWRP_OFF )
	_FOSCSEL	( FNOSC_PRIPLL & IESO_OFF )
	_FOSC		( FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT )
	_FWDT		( FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768 )
	_FPOR		( FPWRT_PWR32 )

	_FUID0		( 0x21 )
	_FUID1		( 0x05 )
	_FUID2		( 0x20 )
	_FUID3		( 0x13 )


/*   Module data   */

#define MENUSW _RG9             /**< Pushbutton: Menu */
#define LD3    _LATG0           /**< LED 3 at port G bit 0 */
#define LD3BIT 0                /**< LED 3 bit */
#define LD3EN  _TRISG0          /**< LED 3 output enable */
#define LD4    _LATG1           /**< LED 4 at port G bit 1 */
#define LD4BIT 1                /**< LED 4 bit */
#define LD4EN  _TRISG1          /**< LED 4 output enable */

static uint8_t scroll;          /* current scroll setting */


/*   Write a stripe row   */

static void stripe(unsigned int row, uint8_t code)
	{
	unsigned int i;

	lcd_pos(row, 59);
	lcd_write(0xff);
	lcd_write(0x00);

	for (i = 0; i < 6; i++)
		lcd_write(code);

	lcd_write(0x00);
	lcd_write(0xff);
	}


/*   Initialize SDR Cube display   */

static void init_display(void)
	{
	unsigned int i;

	scroll = 0;                 /* initialize scroll counter */
	lcd_init();                 /* initialize the display */

	for (i = 0; i < 8; i += 2)
		{
		stripe(i, 0x0f);        /* write even row stripe */
		stripe(i + 1, 0xf0);    /* write odd row stripe */
		}
	}


/**	Target processor initialization
 *
 *	@return @a true if Menu key is pushed
 *
 *	The function initializes the SDR cube hardware for
 *	loader and checks if loading is requested by pushing
 *	the Menu key during power up.
 */

bool target_init(void)
	{
	/* check for Menu button push */

	if (MENUSW)					/* Menu button not pushed: */
		return false;			/*   start user code immediately */

	/* Set up clocking: 38,7072 MHz FCY setup, CPU current 78mA */

	PLLFBDbits.PLLDIV  = 61;    /* Xtal(7,3728MHz) / 3 * 63 = 154,8288MHz */

	/* set PRE and POST scalers */

	CLKDIVbits.PLLPRE  = 1;     /* N1 = 3 */
	CLKDIVbits.PLLPOST = 0;     /* N2 = 2 */

	/* common clock setup */

	CLKDIVbits.FRCDIV  = 4;
	CLKDIVbits.DOZEN   = 0;
	CLKDIVbits.DOZE	   = 0;
	CLKDIVbits.ROI     = 0;

	RCONbits.SWDTEN    = 0;     /* no watchdog */

	do
		;                       /* Wait for PLL to lock */
	while (!OSCCONbits.LOCK);

	LD3EN = 0;                  /* enable LD3 output */
	LD4EN = 0;                  /* enable LD4 output */
	LD3 = 1;                    /* set LD3 for start up indication */
	init_display();             /* initialize display */
	return true;
	}


/**	Report a packet
 *
 *	The function reports a received packet by toggling
 *	LED LD4 and scrolling the pattern on the display
 *	downward by two scan lines.
 */

void report(void)
	{
	/* toggle LD4 to show a packet received */

	asm volatile("btg _LATGbits, #%0" :: "i" (LD4BIT));

	/* scroll display down two scan lines */

	lcd_scroll(scroll -= 2);
	}


