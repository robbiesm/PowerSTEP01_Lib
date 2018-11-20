/*
 * main.h
 *
 * Created: 14/02/2018 22:32:29
 *  Author: HP-wx6600
 */ 


#ifndef MAIN_H_
#define MAIN_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#if defined(__AVR_ATmega2560__)
#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_MOSI	DDB2
#define DD_SCK	DDB1
#define DD_SC0	DDB0
#define SC0		PORTB0

#define LEDinit DDRB |= (1<<DDB7)

#elif defined(__AVR_ATmega328P__)
#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_MOSI	DDB3
#define MOSI	PORTB3
#define DD_MISO	DDB3
#define DD_SCK	DDB5
#define SCK		PORTB5
#define DD_CS0	DDB2
#define CS0		PORTB2

#elif defined(__AVR_ATmega32U4__)
#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define DD_MOSI	DDB2
#define MOSI	PORTB2
#define DD_MISO	DDB3
#define MISO	PORTB3
#define DD_SCK	DDB1
#define SCK		PORTB1
#define DD_CS0	DDB0
#define CS0		PORTB0

#define LEDinit DDRB |= (1<<DDB5)

#endif

#include <avr/cpufunc.h>
#include <avr/io.h>

#define LSBFIRST 0
#define MSBFIRST 1

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01 // SPI2X = bit 0 on SPSR

void SPI_Init(void);
void setBitOrder(uint8_t bitOrder);
void setDataMode(uint8_t dataMode);
void setClockDivider(uint8_t clockDiv);
void SPIdeInit(void);
uint8_t transfer(uint8_t data);
uint16_t transfer16(uint16_t data);

#endif /* MAIN_H_ */