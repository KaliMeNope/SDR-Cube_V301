/**	@file cubeflash.h
 *	@brief SDR Cube flasher definitions
 *	@version 0.03, 21.5.2013
 *	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 */

#ifndef CUBEFLASH_H_

#if defined(__XC16__)
#include <stdbool.h>
#include <stdint.h>
#else
#include "typehelp.h"
#endif

#include <p33FJ256GP710.h>

/*   Attribute shorthand macros   */

#define INLINE inline __attribute__((always_inline))
#define NAKED __attribute__((naked))

/*   cubehw.c   */

extern void report(void);
extern bool target_init(void);

/*   lcd.c   */

extern void lcd_init(void);
extern void lcd_light(bool on);
extern void lcd_on(bool on);
extern void lcd_pos(uint8_t row, uint8_t col);
extern void lcd_scroll(uint8_t topline);
extern void lcd_write(uint8_t data);

/*   serio.c   */

extern uint8_t getb(void);
extern void putb(uint8_t b);
extern bool rxrdy(void);
extern void sio_init(void);

/*   timer.c   */

extern bool timeout(void);
extern void tmr_init(uint8_t seconds);

#define CUBEFLASH_H_
#endif /* CUBEFLASH_H_ */
