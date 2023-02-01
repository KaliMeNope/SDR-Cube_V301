/********************************************************************
* FileName:		main.c
* Dependencies:    
* Processor:	dsPIC33F Family
* Hardware:		SDR Cube
* Compiler:		C30 3.20
* Company:		Microchip Technology, Inc.
*
* Software License Agreement
*
* The software supplied herewith by Microchip Technology Incorporated
* (the “Company”) for its PICmicro® Microcontroller is intended and
* supplied to you, the Company’s customer, for use solely and
* exclusively on Microchip PICmicro Microcontroller products. The
* software is owned by the Company and/or its supplier, and is
* protected under applicable copyright laws. All rights are reserved.
* Any use in violation of the foregoing restrictions may subject the
* user to criminal sanctions under applicable laws, as well as to
* civil liability for the breach of the terms and conditions of this
* license.
*
* THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
* IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
* CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
*
*********************************************************************/

//
// Code adapted to SDR Cube project
// OH2NLT / Juha Niinikoski 11.08.2010
//
// Start up condition (Menu SW) added
// COMM Changed to UART1
// Processor config changed to match SDR Cube start up
//
// Prosessor dsPIC33FJ256GP710
// Hardware SDR-Cube board set
// Dev tools Mplab v8.40, Microchip C30 v3.20
//

#include <p33FJ256GP710.h>

// -------- Note ---------
//
// Processor config must match with the application
// When Flasher is used system start up config come from this module !
//
//
// Flasher is started only if Meter SW is pressed while power on
// LD3 comes on when flasher started
// LD4 toggle with RX command packets
//

// system clock speed increaced to 38.7072 MHz, 13.08.2010


// FCY must be integer multiple of highest baud rate * 16
//#define FCY   40000000					// default
//#define FCY 29491200UL						// 29,4912MHz
#define FCY 38707200UL						// 38,7072 MHz

#define BAUDRATE        115200
//#define BAUDRATE        9600  
                  
#define BRGVAL          ((FCY/BAUDRATE)/16)-1 

// SDR Cube start up config

// processor config
	_FBS		( RBS_NO_RAM & BSS_NO_FLASH & BWRP_WRPROTECT_OFF )
	_FSS		( RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF )
	_FGS		( GSS_OFF & GWRP_OFF )
	_FOSCSEL	( FNOSC_PRIPLL & IESO_OFF )
//	_FOSCSEL	( FNOSC_PRIPLL & IESO_OFF & TEMP_OFF )
	_FOSC		( FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_XT )
	_FWDT		( FWDTEN_OFF & WINDIS_OFF & WDTPRE_PR128 & WDTPOST_PS32768 )
	_FPOR		( FPWRT_PWR32 )
//	_FPOR		( PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR32 )

	_FUID0		( 0x03 )
	_FUID1		( 0x08 )
	_FUID2		( 0x20 )
	_FUID3		( 0x10 )



#define COMMAND_NACK     0x00
#define COMMAND_ACK      0x01
#define COMMAND_READ_PM  0x02
#define COMMAND_WRITE_PM 0x03
#define COMMAND_WRITE_CM 0x07
#define COMMAND_RESET    0x08
#define COMMAND_READ_ID  0x09


#define PM_ROW_SIZE 64 * 8
#define CM_ROW_SIZE 8
#define CONFIG_WORD_SIZE 1

#define PM_ROW_ERASE 		0x4042
#define PM_ROW_WRITE 		0x4001
#define CONFIG_WORD_WRITE	0X4000

typedef short          Word16;
typedef unsigned short UWord16;
typedef long           Word32;
typedef unsigned long  UWord32;

typedef union tuReg32
{
	UWord32 Val32;

	struct
	{
		UWord16 LW;
		UWord16 HW;
	} Word;

	char Val[4];
} uReg32;



// assembler module functions
extern UWord32 ReadLatch(UWord16, UWord16);					// TPLPAG, EA, returd = 32-bit data
extern void WriteLatch(UWord16, UWord16, UWord16, UWord16);	// TPLPAG, EA, data HI, data LO
extern void LoadAddr(UWord16, UWord16);						// NVMADRU, NVMADR
extern void WriteMem(UWord16);								// NVMCON
extern void Erase(UWord16, UWord16, UWord16);
extern void ResetDevice(void);


// local functions
void PutChar(char);
void GetChar(char *);
void WriteBuffer(char *, int);
void ReadPM(char *, uReg32);
void WritePM(char *, uReg32);


char Buffer[PM_ROW_SIZE*3 + 1];


const char ver[] ={"OH2NLT / 11.08.2010"};					// version info

/******************************************************************************/

int main(void)
{
    uReg32 SourceAddr;
	uReg32 Delay;

// SDR Cube clock config

// set PLL divisor
// FCY = Fosc / 2
// FCY must be a "baud rate clock", integer multiple of 1,8432MHz

// 29,4912 MHz FCY setup, CPU current 65mA
#if 0
	PLLFBDbits.PLLDIV	=	30;		// Xtal(7,3728MHz) / 2 * 32 = 117,9648MHz
// set PRE and POST scalers
	CLKDIVbits.PLLPRE	=	0;		// N1 = 2
	CLKDIVbits.PLLPOST	=	0;		// N2 = 2, 117,9648MHz / 2 / 2 = 29,4912 MHz FCY

#endif

// 38,7072 MHz FCY setup, CPU current 78mA
//#if 0
	PLLFBDbits.PLLDIV	=	61;		// Xtal(7,3728MHz) / 3 * 63 = 154,8288MHz
// set PRE and POST scalers
	CLKDIVbits.PLLPRE	=	1;		// N1 = 3
	CLKDIVbits.PLLPOST	=	0;		// N2 = 2, 154,8288MHz / 2 / 2 = 38,7072 MHz FCY
//#endif

// common clock setup
	CLKDIVbits.FRCDIV	=	4;
	CLKDIVbits.DOZEN	=	0;
	CLKDIVbits.DOZE		=	0;
	CLKDIVbits.ROI		=	0;


	RCONbits.SWDTEN = 0;           /* Disable Watch Dog Timer*/

// SDR Cube specific start up things
// set up required I/O for start up condition check	

	_TRISG0 = 0;				// LD3 = output
	_TRISG1 = 0;				// LD4 = output

	asm("nop");
	asm("nop");

	if(_RG9 == 1)				// Meter switch not pressed
		ResetDevice();			// start user code immediatelly


	while(OSCCONbits.LOCK!=1) {}; /* Wait for PLL to lock*/

	_LATG0 = 1;					// set LD3,  Flasher started indication

	
	SourceAddr.Val32 = 0xc00;

	Delay.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);


	if(Delay.Val[0] == 0)
	{
		ResetDevice();
	}


	T2CONbits.T32 = 1; /* to increment every instruction cycle */
	IFS0bits.T3IF = 0; /* Clear the Timer3 Interrupt Flag */
	IEC0bits.T3IE = 0; /* Disable Timer3 Interrup Service Routine */

	if((Delay.Val32 & 0x000000FF) != 0xFF)
	{
		/* Convert seconds into timer count value */
		Delay.Val32 = ((UWord32)(FCY)) * ((UWord32)(Delay.Val[0]));

		PR3 = Delay.Word.HW;
		PR2 = Delay.Word.LW;

		/* Enable Timer */
		T2CONbits.TON=1;
	}


	U1BRG = BRGVAL;      /*  BAUD Rate Setting of Uart2  (0XEF for 9600)*/


	U1MODE = 0x8000; 	/* Reset UART to 8-n-1, alt pins, and enable */
	U1STA  = 0x0400; 	/* Reset status register and enable TX */
   

	while(1)
	{
		char Command;

		GetChar(&Command);
		_LATG1 = !_LATG1;						// toggle LD4,  Flasher receiving data indication


		switch(Command)
		{
			case COMMAND_READ_PM:				/*tested*/
			{
				uReg32 SourceAddr;

				GetChar(&(SourceAddr.Val[0]));
				GetChar(&(SourceAddr.Val[1]));
				GetChar(&(SourceAddr.Val[2]));
				SourceAddr.Val[3]=0;

				ReadPM(Buffer, SourceAddr);

				WriteBuffer(Buffer, PM_ROW_SIZE*3);

				break;
			}

			case COMMAND_WRITE_PM:				/* tested */
			{
			    uReg32 SourceAddr;
				int    Size;

				GetChar(&(SourceAddr.Val[0]));
				GetChar(&(SourceAddr.Val[1]));
				GetChar(&(SourceAddr.Val[2]));
				SourceAddr.Val[3]=0;
				
				for(Size = 0; Size < PM_ROW_SIZE*3; Size++)
				{
				GetChar(&(Buffer[Size]));
				}

				Erase(SourceAddr.Word.HW,SourceAddr.Word.LW,PM_ROW_ERASE);
		
				WritePM(Buffer, SourceAddr);		/*program page */

				PutChar(COMMAND_ACK);				/*Send Acknowledgement */

 				break;
			}

			case COMMAND_READ_ID:
			{
				uReg32 SourceAddr;
				uReg32 Temp;

				SourceAddr.Val32 = 0xFF0000;

				Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);

				WriteBuffer(&(Temp.Val[0]), 4);

				SourceAddr.Val32 = 0xFF0002;

				Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);

				WriteBuffer(&(Temp.Val[0]), 4);

				break;
			}
			
			case COMMAND_WRITE_CM:	
			{
				int    Size;

				for(Size = 0; Size < CM_ROW_SIZE*3;)
				{
					GetChar(&(Buffer[Size++]));
					GetChar(&(Buffer[Size++]));
					GetChar(&(Buffer[Size++]));
				
					PutChar(COMMAND_ACK);				/*Send Acknowledgement */
				}

				
				break;
			}
				case COMMAND_RESET:
			{
				uReg32 SourceAddr;
				int    Size;
				uReg32 Temp;
				

				for(Size = 0, SourceAddr.Val32 = 0xF80000; Size < CM_ROW_SIZE*3; 
																Size +=3, SourceAddr.Val32 += 2)
				{
					if(Buffer[Size] == 0)
					{
						Temp.Val[0]=Buffer[Size+1];
						Temp.Val[1]=Buffer[Size+2];

						WriteLatch( SourceAddr.Word.HW,
										SourceAddr.Word.LW,
										Temp.Word.HW,
										Temp.Word.LW);

						WriteMem(CONFIG_WORD_WRITE);
					}
				}
				

				ResetDevice();

				break;
			}

			case COMMAND_NACK:
			{
				ResetDevice();

				break;
			}

			
			default:
				PutChar(COMMAND_NACK);
				break;
		}

	}

}

/******************************************************************************/
void GetChar(char * ptrChar)
{
	while(1)
	{	
		/* if timer expired, signal to application to jump to user code */
		if(IFS0bits.T3IF == 1)
		{
			* ptrChar = COMMAND_NACK;
			break;
		}
		/* check for receive errors */
		if(U1STAbits.FERR == 1)
		{
			continue;
		}
			
		/* must clear the overrun error to keep uart receiving */
		if(U1STAbits.OERR == 1)
		{
			U1STAbits.OERR = 0;
			continue;
		}

		/* get the data */
		if(U1STAbits.URXDA == 1)
		{
			T2CONbits.TON=0; /* Disable timer countdown */
			* ptrChar = U1RXREG;
			break;
		}
	}
}


/******************************************************************************/
void ReadPM(char * ptrData, uReg32 SourceAddr)
{
	int    Size;
	uReg32 Temp;

	for(Size = 0; Size < PM_ROW_SIZE; Size++)
	{
		Temp.Val32 = ReadLatch(SourceAddr.Word.HW, SourceAddr.Word.LW);

		ptrData[0] = Temp.Val[2];;
		ptrData[1] = Temp.Val[1];;
		ptrData[2] = Temp.Val[0];;

		ptrData = ptrData + 3;

		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}
}
/******************************************************************************/

void WriteBuffer(char * ptrData, int Size)
{
	int DataCount;
	
	for(DataCount = 0; DataCount < Size; DataCount++)
	{
		PutChar(ptrData[DataCount]);
	}
}
/******************************************************************************/
void PutChar(char Char)
{
	while(!U1STAbits.TRMT);
	
	U1TXREG = Char;
}
/******************************************************************************/

void WritePM(char * ptrData, uReg32 SourceAddr)
{
	int    Size,Size1;
	uReg32 Temp;
	uReg32 TempAddr;
	uReg32 TempData;

	for(Size = 0,Size1=0; Size < PM_ROW_SIZE; Size++)
	{
		
		Temp.Val[0]=ptrData[Size1+0];
		Temp.Val[1]=ptrData[Size1+1];
		Temp.Val[2]=ptrData[Size1+2];
		Temp.Val[3]=0;
		Size1+=3;

	   WriteLatch(SourceAddr.Word.HW, SourceAddr.Word.LW,Temp.Word.HW,Temp.Word.LW);

		/* Device ID errata workaround: Save data at any address that has LSB 0x18 */
		if((SourceAddr.Val32 & 0x0000001F) == 0x18)
		{
			TempAddr.Val32 = SourceAddr.Val32;
			TempData.Val32 = Temp.Val32;
		}

		if((Size !=0) && (((Size + 1) % 64) == 0))
		{
			/* Device ID errata workaround: Reload data at address with LSB of 0x18 */
	      WriteLatch(TempAddr.Word.HW, TempAddr.Word.LW,TempData.Word.HW,TempData.Word.LW);

			WriteMem(PM_ROW_WRITE);
		}

		SourceAddr.Val32 = SourceAddr.Val32 + 2;
	}

   
}

/******************************************************************************/



