/**	@file target.h
 *	@brief AN1094 target protocol definitions
 *	@version 0.01, 25.4.2013
 *	@author Tauno Voipio, OH2UG, oh2ug@iki.fi
 *
 *	This file contains the definitions for the
 *	Microchip AN1094 host - target protocol.
 *
 *
 * @par Target protocol
 *
 * The communication between the controlling processor
 * (PC host) and the flash writing agent in the target
 * dsPIC uses asynchronous serial communication with
 * a RS-232 -style interface, a serial port at the host
 * and an UART at the target.
 *
 * The first byte of each packet sent determines the
 * type of the packet, its length and expected response
 * packet size and contents. The packet type bytes are
 *  in enum @a ptype.
 *
 * The addresses sent in command packets are 24 bit program
 * addresses, the same addresses as used by the processor
 * program counter.
 *
 * @par Read program memory
 *
 * To read a page of program memory, the host sends
 * a request packet of four bytes:
 *  - Command: C_READ_PM (0x02),
 *  - Program address, least significant byte
 *  - Program address, middle byte
 *  - Program address, most significant byte
 *
 * The program address must be on a page boundary, (a multiple
 * of 0x400 in a dsPIC33).
 *
 * The expected response is a page (512 24 bit words in a dsPIC33)
 * of memory contents. The data comes in three byte groups:
 *  - Flash data, most significant byte,
 *  - Flash data, middle byte,
 *  - Flash data, least significant byte.
 *
 *  There will be totally 3 * page size of bytes (1536 for dsPIC33).
 *
 *  The command can be used for reading the configuration words.
 *  The response has the configuration words and the user
 *  words in the first 12 groups of three bytes. The rest will be
 *  irrelevant, mostly zeroes (0x00).
 *
 *	@note
 *  Please note that the byte order is different from all
 *  other messages.
 *
 * @par Write program memory
 *
 * To write a page of program memory, the host sends
 * a header packet of four bytes, and the data to be written
 * (1536 bytes, as above, but different byte order):
 * 	- Command: C_WRITE_PM (0x03),
 * 	- Program address, least significant byte
 * 	- Program address, middle byte
 * 	- Program address, most significant byte
 * 	- The data to be written, in three byte groups:
 * 	- -First flash data, least significant byte
 * 	- -First flash data, middle byte
 * 	- -First flash data, most significant byte
 * 	- -Second flash data, least significant byte
 * 	- -Second flash data, middle byte
 * 	- -Second flash data, most significant byte
 * 	- ...
 * 	- -Last flash data, least significant byte
 * 	- -Last flash data, middle byte
 * 	- -Last flash data, most significant byte
 *
 * The program address must be on a page boundary, (a multiple
 * of 0x400 in a dsPIC33).
 *
 * The expected response is one byte:
 *  - C_ACK (0x01), if the write was successful
 *  - C_NACK (0x00), if the write was unsuccessful
 *
 * The write operation erases the target page before writing, so
 * all data previously on the page will be lost.
 *
 * @par Write configuration words
 *
 * To write the configuration words, first all words
 * are transferred to the target, one by one. After
 * the transfer, the configuration words will be written
 * and the target will be reset with a single command.
 * It is not possible to write the configuration words
 * without resetting the target.
 *
 * To transfer the words to the target, a command byte
 * is written first, and eight groups of three bytes are
 * then transferred.
 *
 * The command byte is:
 *  - Command byte: C_WRITE_CM (0x07)
 *
 * No response is expected for the command byte.
 *
 * The three byte words are sent:
 *  - Data empty byte, needs to be 0x00 to enable writing of this word
 *  - Configuration, least significant byte
 *  - Configuration, most significant byte (0xff on dsPIC33)
 *
 * The expected response is one byte:
 *  - C_ACK (0x01), if the write was successful
 *  - C_NACK (0x00), if the write was unsuccessful (not used)
 *
 * The configuration is committed by sending one byte:
 *  - Command byte: C_RESET (0x08)
 *
 * No response is expected. The target is reset.
 *
 * There must be no other data transfers between the configuration
 * setup and commit/reset command.
 *
 * @par Reset target
 *
 * To reset the target without writing configuration, the host
 * sends a single byte:
 *  - Command byte: C_NACK (0x00)
 *
 * No response is expected, and the target will be reset.
 *
 * @par Read processor identifier
 *
 * To identify the target processor, the host sends a single
 * byte command:
 *  - Command: C_READ_ID (0x09)
 *
 * The expected response is eight bytes:
 *  - Processor identifier, least significant byte
 *  - Processor identifier, most significant byte
 *  - Zero (0x00)
 *  - Zero (0x00)
 *  - Version identifier, least significant byte
 *  - Version identifier, most significant byte
 *  - Zero (0x00)
 *  - Zero (0x00)
 *
 */

#ifndef TARGET_H_

/**	Target protocol packet types */

enum ptype
	{
	C_NACK = 0,                /**< negative acknowledge, reset target */
	C_ACK,                     /**< acknowledge */
	C_READ_PM,                 /**< read program memory */
	C_WRITE_PM,                /**< write program memory */
	C_READ_EE,                 /**< read EEPROM (dsPIC30 only) */
	C_WRITE_EE,                /**< write EEPROM (dsPIC30 only) */
	C_READ_CM,                 /**< read configuration memory (not used) */
	C_WRITE_CM,                /**< write configuration memory */
	C_RESET,                   /**< commit configuration and reset target */
	C_READ_ID                  /**< read identifier words */
	};

#define TARGET_H_
#endif /* TARGET_H_ */
