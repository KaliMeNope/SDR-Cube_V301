/**	@file typehelp.h
 *	@brief SDR Cube flasher additional type definitions
 *	@version 0.01, 21.5.2013
 *	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 *	This is a file to replace portions of stdbool.h and
 *	stdint.h for C30 compiler lacking the standard headers.
 */


#ifndef TYPEHELP_H_

/*   Types from stdint.h   */

#ifndef uint8_t
typedef unsigned char uint8_t;
#define uint8_t uint8_t
#endif

#ifndef uint16_t
typedef unsigned int uint16_t;
#define uint16_t uint16_t
#endif

#ifndef uint32_t
typedef unsigned long int uint32_t;
#define uint32_t uint32_t
#endif

/*   Definitions from stdbool.h   */

#ifndef __bool_true_and_false_are_defined

#define bool uint8_t
#define false 0
#define true 1

#define __bool_true_and_false_are_defined
#endif

#define TYPEHELP_H_
#endif /* TYPEHELP_H_ */
