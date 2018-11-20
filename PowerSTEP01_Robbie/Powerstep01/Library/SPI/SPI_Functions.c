/*
 * SPI_Lib.c
 *
 * Created: 14/02/2018 22:31:19
 * Author : HP-wx6600
 */ 


#include "SPI.h"

/* 
* Initializes the ATMEL SPI controller as:
* SPI Master
* Mode3
* F_CPU/8 for 2MHz communication
* Enables SPI
* Set Mosi and Clk pins as output
*/
void SPI_Init(void)
{
	//initialize IO ports
	DDR_SPI |= (1<<DD_MOSI) | (1<<DD_SCK);

	//enable SPI master, set clock Phase and polarity for SPI Mode 3 and select F_CPU/8
	SPCR |= (1<<MSTR) | (1<<SPE) | (1<<CPOL) | (1<<CPHA) | (1<<SPR0);
	SPSR |= (1<<SPI2X);
}
/* 
* Sets Bitorder 
*/
void setBitOrder(uint8_t bitOrder)
{
	if (bitOrder == LSBFIRST) SPCR |= (1<<DORD);
	else SPCR &= ~(1<<DORD);
}
/*
* Sets Data Mode (See p.183 table 17-2 CPOL and CPHA Functionality in datasheet)
* Mode 0 -> CPOL = 0, SPHA = 0
* Mode 1 -> CPOL = 0, SPHA = 1
* Mode 2 -> CPOL = 1, SPHA = 0
* Mode 3 -> CPOL = 1, SPHA = 1
*/
void setDataMode(uint8_t dataMode)
{
	SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;
}
/*
* Sets the clock divider
*/
void setClockDivider(uint8_t clockDiv)
{
    SPCR = (SPCR & ~SPI_CLOCK_MASK) | (clockDiv & SPI_CLOCK_MASK);
    SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((clockDiv >> 2) & SPI_2XCLOCK_MASK);	
}
/*
* Deinitializes the SPI controller
*/
void SPIdeInit(void)
{
	//disable SPI
	SPCR &= ~(1<<SPE);
}
/*
* Transfers one byte over the SPI bus
*/
uint8_t transfer(uint8_t data)
{
	SPDR = data;
	/*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
	 * information from Arduino SPI.h library
     */
	_NOP();
	
	//wait for transmission to end
	while(!(SPSR & (1<<SPIF)));
	
	
	return SPDR;
}
/*
* Transfers two byte over the SPI bus
*/
uint16_t transfer16(uint16_t data)
{
	//special data type that allows to store different data types in 1 memory location
	//only 1 value at the time
	union
	{
		uint16_t val;
		struct { uint8_t lsb; uint8_t msb; };
	} in, out;
	
	in.val = data;
	if (!(SPCR & (1<<DORD))) {
		SPDR = in.msb;
		_NOP(); // See transfer(uint8_t) function
		while(!(SPSR & (1<<SPIF)));
		
		out.msb = SPDR;
		SPDR = in.lsb;
		_NOP();
		while(!(SPSR & (1<<SPIF)));
		out.lsb = SPDR;
	}
	else
	{
		SPDR = in.lsb;
		_NOP();
		while(!(SPSR & (1<<SPIF)));
		
		out.lsb = SPDR;
		SPDR = in.msb;
		_NOP();
		while(!(SPSR & (1<<SPIF)));
		out.msb = SPDR;
	}
	return out.val;	
}
