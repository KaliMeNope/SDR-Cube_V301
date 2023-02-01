This is Beta version of Bootloader for 33F devices.
It was developed on Explorer16 board running 33FJ256GP710 processor.


****************************************************************************
Procedure:

1.	Compile and Load target side bootloader application onto Explorer16 board
3.	Start/Run target side bootloader application
4.	Start/Run host side bootloader application with appropriate arguments from the command line window.
	Executable is located in the "x86 host\Debug" directory
5.	User application must be modified to start at address 0xC00 (see below)

When bootloader runs on the target, it waits N numbers of seconds for UART communication before
transferring control to the user application at address 0xC02. The delay of N seconds must be specified
at address 0xC00. if the delay is 0 seconds, bootloader jumps to user application on the start-up.
If the delay is 255 seconds (oxFF), bootloader will wait for UART communication indefinitely.


****************************************************************************
Usage: "16-Bit Flash Programmer.exe" -i interface [-bpe] hexfile

Options:

  -i
       specifies serial interface name such as COM1, COM2, etc

  -b
       specifies baud rate for serial interface. Default is 115200

  -p
       read program flash. Must provide address to read in HEX format: -p 0x000100

  -e
       read EEPROM. Must provide address to read in HEX format: -e 0x7FFC00


****************************************************************************
GLD file modifications to place application at address 0xC00 and to provide bootloader timeout:

-	Modify the following lines of of GLD code:

  program (xr) : ORIGIN = 0xC00,         LENGTH = 0x29E00
  __CODE_BASE   = 0xC00;    /* Handles, User Code, Library Code */


  /*
  ** User Code and Library Code
  */
  .text __CODE_BASE :
  {
        SHORT(0x0A); /* Bootloader timeout in sec */


        *(.handle);
        *(.libc) *(.libm) *(.libdsp);  /* keep together in this order */
        *(.lib*);
        *(.text);
  } >program


****************************************************************************

