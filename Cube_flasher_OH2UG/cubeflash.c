/**	@file cubeflash.c
 * 	@brief SDR Cube flasher, compatible with Microchip AN1094
 * 	@version 0.05, 22.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 *	@mainpage
 *	This program is a SDR Cube flash reprogramming agent
 *	using the protocol of Microchip AN1094.
 *
 *	For the protocol details, see the documentation in
 *	the file target.h.
 *
 *  This is complete rewrite to be upward compatible with
 *  the Microchip AN1094 flashing agent.
 *
 *  The main differences are:
 *   - Separated serial I/O to own module, serio.c,
 *   - Separated timer handling to own module, timer.c,
 *   - Separated SDR Cube handling to own module, cubehw.c,
 *   - Added more stringent range checks to program areas,
 *   - Added startup vector protection to page 0 programming,
 *   - Completely empty rows (all 0xff) are not programmed,
 *   - Added LCD progress display.
 *
 *	@par Displays
 *
 *	The program switches the display on with a vertical progress
 *	bar in the display. The progress bar is scrolled downward on
 *	each packet handled.
 *
 *	On the circuit board, LED 3 is switched on after starting
 *	up the program. LED 4 will be toggled on each packet handled.
 *
 *	@par Compiling
 *
 *	The code is compiled using the Microchip XC16 v. 1.11 toolset.
 *	It compiles also with C30. The file typehelp.h in needed with
 *	C30 to replace the missing standard headers.
 *
 *	If you recompile the code, please check from the link map
 *	or disassembly (make dis) that the code fits below address 0xc00.
 *
 * 	@par Credits
 *
 * 	Credits to:
 * 	- Creators of Microchip AN1094,
 * 	- Juha Niinikoski, OH2NLT.
 *
 *	@copyright
 *
 *	Copyright (C) 2013 by Tauno Voipio
 *
 *	This program is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	Contact:
 *	@par
 *		Tauno Voipio			@n
 *		Ritokalliontie 8-16 O	@n
 *		FI-00330 Helsinki		@n
 *		Finland
 *
 *	Email:
 *	@par
 *		oh2ug@iki.fi
 */

#include "cubeflash.h"
#include "cubepic.h"
#include "target.h"


/*   Module data   */

#define CF_WRITE 0X4000        /**< NVMCON: write configuration word */
#define NV_WRITE 0x4001        /**< NVMCON: write flash row */
#define NV_ERASE 0x4042        /**< NVMCON: erase flash page */

#define NVM_WR 15              /**< NVMCON: write/erase active bit */

#define NVM_KEY1 0x55          /**< NVMKEY: first write key */
#define NVM_KEY2 0xaa          /**< NVMKEY: second write key */

static uint8_t buf[PGSZ * 3];  /* programming page buffer */

const char ver[] ="OH2UG / 22.05.2013";		/* version info */


/*   Read a 24 bit instruction word from flash   */

static uint32_t tblread(uint32_t loc)
	{
	__asm__ volatile
		(
		"push TBLPAG\n\t"       /* save top address register */
		"mov %d0,TBLPAG\n\t"    /* set up top address */
		"tblrdh [%0], %d0\n\t"  /* read 8 high bits */
		"tblrdl [%0], %0\n\t"   /* read 16 low bits */
		"pop TBLPAG"            /* restore top address register */
		: "=r" (loc)
		);

	return loc;
	}


/*   Load a 24 bit instruction word to flash write latches   */

static void tblwrite(uint32_t loc, uint32_t val)
	{
	__asm__ volatile
		(
		"push TBLPAG\n\t"       /* save top address register */
		"mov %d0,TBLPAG\n\t"    /* set up top address */
		"tblwth %d1, [%0]\n\t"  /* set up 8 top bits */
		"tblwtl %1, [%0]\n\t"   /* set up 16 low bits */
		"pop TBLPAG"            /* restore top address register */
		:
		: "r" (loc),
		  "r" (val)
		);
	}


/*   Do a flash write / erase operation   */

static void burn(uint16_t cmd)
	{
	NVMCON = cmd;               /* set up write/erase command */

	__asm__ volatile
		(
		"disi #5\n\t"           /* no interrupts in sequence */
		"mov #%0,w0\n\t"        /* get first key */
		"mov w0,NVMKEY\n\t"     /* send first key to lock */
		"mov #%1,w0\n\t"        /* get second key */
		"mov w0,NVMKEY\n\t"     /* send second key to lock */
		"bset NVMCON,#%2\n\t"   /* start writing */
		"nop\n\t"               /* wait for flash */
		"nop\n"                 /*  reaction time */
		"1:\n\t"
		"btsc NVMCON,#%2\n\t"   /* done? */
		"bra 1b"                /*   no - loop till done */
		:
		: "i" (NVM_KEY1),
		  "i" (NVM_KEY2),
		  "i" (NVM_WR)
		);
	}


/*   Go to client code   */

static INLINE void go(void)
	{
	__asm__ volatile("goto %0" :: "i" (USER_BASE + 2));
	}


/*   Send a block to serial line   */

static void putblk(uint8_t *bp, unsigned int count)
	{
	for ( ; count != 0; count--)
		putb(*bp++);
	}


/*   Get a block from serial line   */

static void getblk(uint8_t *bp, unsigned int count)
	{
	for ( ; count != 0; count--)
		*bp++ = getb();
	}


/*   Get a 24 bit address from line, least significant byte first   */

static uint32_t get24(void)
	{
	union
		{
		uint32_t d;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	getblk(xlt.b, 3);
	xlt.b[3] = 0;

	return xlt.d;
	}


/*   Send a 32 bit word from flash to line   */

static void put32(uint32_t loc)
	{
	union
		{
		uint32_t d;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	xlt.d = tblread(loc);
	putblk(xlt.b, sizeof(xlt));
	}


/*   Read a page from flash into buffer   */

static void readpage(uint32_t loc)
	{
	unsigned int i;
	uint8_t *dp;
	union
		{
		uint32_t d;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	dp = buf;

	for (i = 0; i < PGSZ; loc += 2, i++)
		{
		xlt.d = tblread(loc);

		/* get in big-endian format (unusual), for compatibility */

		*dp++ = xlt.b[2];
		*dp++ = xlt.b[1];
		*dp++ = xlt.b[0];
		}
	}


/*   Write an instruction from buffer to latches   */

 static bool writeinst(uint32_t loc)
	{
	uint8_t *p;
	union
		{
		uint32_t d;
		uint16_t w;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	/* instruction number in page is (loc / 2) % PGSZ */
	/* make a pointer into buffer slot */
	/* casting to 16 bits simplifies the code and preserves the result */

	p = &buf[3 * (((uint16_t)loc / 2) % PGSZ)];

	xlt.b[0] = p[0];
	xlt.b[1] = p[1];
	xlt.b[2] = p[2];
	xlt.b[3] = 0;

	tblwrite(loc, xlt.d);

	/* return true if at least one byte needs to be programmed */

	return xlt.w != 0xffff || xlt.b[2] != 0xff;
	}


/*   Program a row from buffer   */

static void programrow(uint32_t loc)
	{
	unsigned int i;
	bool nonempty;

	nonempty = false;

	/* fill latches */

	for (i = 0; i < 2 * ROWSZ; i += 2)
		{
		if (writeinst(loc + i))
			nonempty = true;
		}

	/* program only non-empty rows */

	if (nonempty)
		{
		/* workaround for chip ID error */

		writeinst(loc + 0x18);

		/* burn the row */

		burn(NV_WRITE);
		}
	}


/*   Patch an instruction from flash into page buffer   */

static void patch(uint32_t loc, unsigned int off)
	{
	uint8_t *p;
	union
		{
		uint32_t d;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	xlt.d = tblread(loc);
	p = &buf[off];

	*p++ = xlt.b[0];
	*p++ = xlt.b[1];
	*p   = xlt.b[2];
	}


/*   Program a page from buffer   */

static void programpage(uint32_t loc)
	{
	unsigned int r;

	/* check for startup linkage page, protect linkage */

	if (loc == 0)
		{
		patch(0, 0);    /* first instruction word */
		patch(2, 3);    /* second instruction word */
		}

	/* erase the page */

	tblwrite(loc, 0);
	burn(NV_ERASE);

	/* program it, row by row */

	for (r = 0; r < PGWSZ; r += 2 * ROWSZ)
		programrow(loc + r);
	}


/*   Save configuration words into buffer   */

static void saveconf(void)
	{
	unsigned int i;

	for (i = 0; i < NCONF * 3; i += 3)
		{
		getblk(&buf[i], 3);
		putb(C_ACK);				/* acknowledge each word */
		}
	}


/*   Program saved configuration words from buffer   */

static void programconf(void)
	{
	unsigned int i;
	uint32_t loc;
	union
		{
		uint32_t d;
		uint8_t b[sizeof(uint32_t)];
		} xlt;

	loc = CONF_BASE;
	xlt.d = 0;

	for (i = 0; i < NCONF * 3; loc += 2, i += 3)
		{
		if (buf[i] == 0)
			{
			xlt.b[0] = buf[i + 1];
			xlt.b[1] = buf[i + 2];

			tblwrite(loc, xlt.d);
			burn(CF_WRITE);
			}
		}
	}


/** Main program
 *
 *	@return None, no return
 *
 *	This is the main program, with an eternal loop
 *	without return. Stand-alone code, no arguments.
 */

int main(void)
	{
	uint32_t loc;
	uint8_t cmd;

	if (!target_init())
		go();

	cmd = tblread(USER_BASE);

	if (cmd == 0)
		go();

	tmr_init(cmd);
	sio_init();

	/* wait for master, exit to client if timeout */

	do
		{
		if (timeout())
			go();
		}
	while (!rxrdy());

	/* main loop, interpret the line protocol */

	for (;;)
		{
		cmd = getb();
		report();

		switch (cmd)
			{
			case C_READ_PM:
				readpage(get24());
				putblk(buf, sizeof(buf));
				break;

			case C_WRITE_PM:
				loc = get24();
				getblk(buf, sizeof(buf));

				/* check that location is on a page boundary */

				if (((uint16_t)loc % PGWSZ) == 0)
					{
					programpage(loc);
					putb(C_ACK);        /* acknowledge */
					}
				else
					putb(C_NACK);       /* not page boundary, reject */

				break;

			case C_READ_ID:
				put32(IDNT_BASE);
				put32(IDNT_BASE + 2);
				break;
			
			case C_WRITE_CM:
				saveconf();
				break;

			case C_RESET:
				programconf();
				go();
				break;

			case C_NACK:
				go();
				break;

			default:
				putb(C_NACK);
				break;
			}
		}
	}
