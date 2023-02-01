/**	@file lcd.c
 * 	@brief SDR Cube LCD driver, stripped
 * 	@version 0.03, 22.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 * 	The module is a simple driver for the
 * 	Crystalfonz CFAG12864B-TFH-V 128 * 64 display with
 * 	the Samsung S6B0108 / Neotec NT7108 Controller.
 *
 * 	@par Credits
 *
 * 	Credits to:
 * 	- Milton E. Cram, W8NUE
 * 	- George L. Heron, N2APB
 * 	- Juha Niinikoski, OH2NLT
 *
 * 	@par Display layout
 *
 * 	The display is 8 rows of 8 bytes (64 pixels vertical)
 * 	by 128 columns horizontal. The vertical bytes have the
 * 	least significant bit at top. The horizontal columns are
 * 	divided between the two controller chips. The driver
 * 	handles the chip selection and stepping from left chip
 * 	to right chip.
 */

#include "cubeflash.h"


/*    Module data    */

#define DISP_BL	 _LATF13         /**< Display backlight, 1 = on */
#define BL_BIT   (1 << 13)       /**< Backlight bit in port F */

#define LCD_TRIS TRISE           /**< LCD data bus output enable */
#define	LCD_OUT  LATE            /**< LCD data bus out */
#define LCD_IN   PORTE           /**< LCD data bus in */

#define	LCD_CS1  LATDbits.LATD7  /**< Left controller chip select */
#define	LCD_CS2  LATDbits.LATD6  /**< Right controller chip select */
#define	LCD_RW   LATDbits.LATD5  /**< R/W- controller input */
#define	LCD_DI   LATDbits.LATD4  /**< D/I- controller input */
#define	LCD_E    LATDbits.LATD3  /**< Controller transfer strobe (enable) */
#define CTL_BITS (0x1f << 3)     /**< Control signal mask in port D */

#define DSP_OFF  0x3e            /**< Command: display off */
#define DSP_ON   0x3f            /**< Command: display on */
#define SET_COL  0x40            /**< Command: set column (6 bits of column) */
#define SET_ROW  0xb8            /**< Command: set row (3 bits of row) */
#define SET_TOP  0xc0            /**< Command: set top line (6 bits of line) */

#define S_BSY    (1 << 7)        /**< Status: busy */
#define S_ON     (1 << 5)        /**< Status: display is on */
#define S_RST    (1 << 4)        /**< Status: reset */

#define COLUMNS  128             /**< Number of columns (both together) */
#define ROWS     8               /**< Number of byte rows */
#define LINES    (8 * ROWS)      /**< Number of scan lines */

#define RIGHT	(1 << 0)         /**< Right controller selector bit */
#define LEFT	(1 << 1)         /**< Left controller selector bit */
#define	BOTH	(LEFT | RIGHT)   /**< Selector for both controllers */

static uint8_t column; 	         /* current column on display */


/*   Address setup delay, about 150 ns   */

static INLINE void dly_asu(void)
	{
	__asm__ volatile
		(
		"repeat #5 - 1\n\t"
		"nop"
		);
	}


/*   Write and enable pulse hold delay, about 450 ns   */

static INLINE void dly_hld(void)
	{
	__asm__ volatile
		(
		"repeat #17 - 1\n\t"
		"nop"
		);
	}


/*   Enable pulse width delay, about 500 ns   */

static INLINE void dly_e(void)
	{
	__asm__ volatile
		(
		"repeat #20 - 1\n\t"
		"nop"
		);
	}


/*   Set chip selects   */

static void setcs(unsigned int select)
	{
	if (select & LEFT)
		LCD_CS1 = 0;
	else
		LCD_CS1 = 1;

	if (select & RIGHT)
		LCD_CS2 = 0;
	else
		LCD_CS2 = 1;
	}


/*   Wait until the chip is not busy   */

static void pollbsy(unsigned int select)
	{
	if (select != 0)
		{
		setcs(select);          /* select chip */
		dly_asu();              /* wait CS setup time */
		LCD_E = 1;              /* start reading */
		dly_e();                /* wait data setup time */

		do
			;
		while (LCD_IN & S_BSY);  /* loop until not busy */

		LCD_E = 0;              /* end strobe */
		}
	}


/*   Wait until controller(s) not busy   */

static void waitbsy(unsigned int select)
	{
	LCD_TRIS = ~0;              /* data port in */
	LCD_DI = 0;                 /* select status register */
	LCD_RW = 1;                 /* reading */

	pollbsy(select & LEFT);     /* poll left controller */
	pollbsy(select & RIGHT);    /* poll right controller */
	}


/*   Write to selected controller port   */

static void writeout(uint8_t data)
	{
	LCD_RW = 0;					/* set write */
	LCD_OUT = data; 			/* load data to be written */
	LCD_TRIS = 0;               /* data port out */
	dly_asu();				    /* address setup time, tASU 140ns */

	LCD_E = 1;					/* start write */
	dly_e();				    /* wait strobe pulse width */
	LCD_E = 0;					/* write done */

	dly_hld();                  /* ensure E low and data hold times */
	}


/*   Write to control register(s)   */

static void write_ctl (unsigned int select, uint8_t ctl)
	{
	waitbsy(select);            /* wait for chip(s) */
	setcs(select);              /* select the chip(s) */
	writeout(ctl);              /* send the control out */
	}


/*   Pick controller selector from column number */

static unsigned int get_select(uint8_t col)
	{
	if (col < COLUMNS / 2)
		return LEFT;

	return RIGHT;
	}


/*   Select display row   */

static void set_row (uint8_t row)
	{
	write_ctl(BOTH, SET_ROW | row);
	}


/*   Select controller column   */

static void set_column(uint8_t col)
	{
	write_ctl(get_select(col), SET_COL | col % (COLUMNS / 2));
	column = col;
	}


/**   Set display position
 *
 * @param row Byte row for next write
 * @param col Column for next write
 *
 * The function sets the next write position to
 * the byte row @a row and bit column @a col. The
 * range of rows is 0 to 7 and columns 0 to 127.
 * The @a row 0 is at the top of the display,
 * unless the display is scrolled. The @a col 0
 * is at the left edge of the display.
 */

void lcd_pos(uint8_t row, uint8_t col)
	{
	set_row(row % ROWS);
	set_column(col % COLUMNS);
	}


/**   Set vertical scroll register
 *
 * @param topline Display memory pixel line at top
 *
 * The function selects the display memory pixel
 * line to be shown at top of the display. The
 * value 0 selects an unscrolled display. The range
 * of @a topline is 0 to 63. Increasing @a topline
 * moves the displayed image up.
 */

void lcd_scroll(uint8_t topline)
	{
	write_ctl(BOTH, SET_TOP | (topline % LINES));
	}


/**   Write to display
 *
 * @param data Byte to the display
 *
 * The function writes the byte @a data to
 * the display position selected by lcd_pos().
 * The position is incremented to the right and
 * wrapped to the left edge from the right edge.
 * The byte is with least significant bit at top.
 * A bit value of 1 creates a bright (white) pixel and
 * a bit value of 0 creates a dark (blue) pixel.
 */

void lcd_write(uint8_t data)
	{
	/* select controller and wait for it */

	waitbsy(get_select(column));

	/* write */

	LCD_DI = 1;                 /* select data register */
	writeout(data);             /* write the data */

	/* increment column, keep in range */

	column++;                   /* increment column */
	column %= COLUMNS;          /* keep in display area */

	/* switch controller, if stepped to new half */

	if (column == 0 || column == COLUMNS / 2)
		set_column(column);
	}


/** Clear display
 *
 * @param row Byte row to start the clear
 *
 * The function clears the display starting from
 * byte row @a row to the bottom of the display.
 * The function also clears the scroll to 0.
 * The write position is reset to top left corner.
 */

void lcd_clear(uint8_t row)
	{
	uint8_t i;

	lcd_scroll(0);

	for ( ; row < ROWS; row++)
		{
		lcd_pos(row, 0);

		for (i = 0; i < COLUMNS; i++)
			lcd_write(0);
		}

	lcd_pos(0, 0);
	}


/**  Switch display on or off
 *
 * @param on The desired on/off state
 *
 * The function switches the display off if
 * @a on is @a false (zero). It switches the
 * display on if @a on is @a true (non-zero),
 */

void lcd_on(bool on)
	{
	write_ctl(BOTH, (on) ? DSP_ON : DSP_OFF);
	}


/**  Switch display backlight on or off
 *
 * @param on The desired on/off state
 *
 * The function switches the display backlight
 * off if @a on is @a false (zero). It switches
 * the backlight on if @a on is @a true (non-zero).
 */

void lcd_light(bool on)
	{
	if (on)
		DISP_BL = 1;
	else
		DISP_BL = 0;
	}


/**	Initialize the display
 *
 * 	The function initializes the display,
 * 	clears it, resets scroll to 0 and switches
 * 	the display on.
 */

void lcd_init(void)
	{
	LCD_E	 = 0;               /* strobe off */
	DISP_BL  = 0;               /* light off */

	AD1PCFGH |= 0xff00;         /* enable digital inputs from LCD bus */
	TRISD    &= ~CTL_BITS;      /* enable control outputs */
	TRISF    &= ~BL_BIT;        /* enable backlight control */

	lcd_clear(0);               /* clear whole display */
	lcd_on(true);               /* switch it on */
	lcd_light(true);            /* switch light on */
	}
