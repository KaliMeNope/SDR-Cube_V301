/**	@file cubepic.h
 * 	@brief SDR Cube dsPIC definitions
 * 	@version 0.02, 18.5.2013
 * 	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 * 	This file contains the definitions for the
 * 	dsPIC33FJ256GP710 processor in SDR Cube.
 */

#ifndef CUBEPIC_H_

/*   Memory range sizes   */

#define NCONF 8                /**< Configuration words */
#define NROWS 8                /**< Rows in an erase page */
#define ROWSZ 64               /**< Instructions in a programming row */
#define PGSZ  (NROWS * ROWSZ)  /**< Erase page size, instructions */
#define PGWSZ (2 * PGSZ)       /**< Page size in two-byte words */
#define PGBSZ (2 * PGWSZ)      /**< Programming page byte size */
#define CONF_SIZE (2 * NCONF)  /**< Configuration area size, two-byte words */
#define IDNT_SIZE 2            /**< Identification register size, two-byte words */

/*   Memory range base addresses   */

#define RSVD_BASE PGWSZ        /**< Flasher reserved area base, two-byte words */
#define USER_BASE (3 * PGWSZ)  /**< User area start, two-byte words */
#define PMEM_SIZE 0x2ac00      /**< Flash size, two-byte words */
#define CONF_BASE 0xf80000     /**< Configuration register base, two-byte words */
#define IDNT_BASE 0xff0000     /**< Identification register base */

/*   Clocking rates   */

#define FCY 38707200L          /**< Master clock frequency, Hz */
#define BRATE 115200           /**< Serial I/O bit rate */

#define CUBEPIC_H_
#endif /* CUBEPIC_H_ */
