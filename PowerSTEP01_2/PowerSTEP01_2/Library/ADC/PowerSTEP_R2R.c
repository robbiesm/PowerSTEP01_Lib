/*
 * PowerSTEP_R2R.c
 *
 * Created: 17/04/2018 13:07:45
 *  Author: vincent
 */ 

#include "powerSTEP_R2R.h"

void ADCInit(void)
{
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX &= ~(1 << ADLAR); // 10 bit reading (left adjust)
	ADMUX &= ~(1 << MUX0); // set to pin 0 of port C
	ADMUX &= ~(1 << MUX1); // set to pin 0 of port C
	ADMUX &= ~(1 << MUX2); // set to pin 0 of port C
	ADMUX &= ~(1 << MUX3); // set to pin 0 of port C
	ADMUX &= ~(1 << MUX4); // set to pin 0 of port C
	ADCSRB &= ~(1 << MUX5); // set to pin 0 of port C
	ADCSRA |= (1 << ADEN);  // Enable ADC
}

void getADC(void)
{
	uint16_t adcval;
	ADCSRA |= (1 << ADSC); //start conversion
	while(ADCSRA & (1<<ADSC));	//wait until conversion is complete
	adcval = ((ADCH & 0b00000011) << 8);
	adcval |= ADCL;
	
	if(adcval < 32)							  //  0 0 0 0 / 0 - 1023
	{
		SWR &= 0b11110000;					//general purpose register, keep 4 MSB's, clear 4 LSB's
		SWR |= 0b00000000;					//use 4 LSB's for switch values
	}
	if((32 < adcval) && (adcval < 96))        //  0 0 0 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000001;
	}
	if((96 < adcval) && (adcval < 160))        //  0 0 1 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000010;
	}
	if((160 < adcval) && (adcval < 224))        //  0 0 1 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000011;
	}
	if((224 < adcval) && (adcval < 288))        //  0 1 0 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000100;
	}
	if((288 < adcval) && (adcval < 352))        //  0 1 0 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000101;
	}
	if((352 < adcval) && (adcval < 416))        //  0 1 1 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000110;
	}
	if((416 < adcval) && (adcval < 480))        //  0 1 1 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00000111;
	}
	if((480 < adcval) && (adcval < 544))        //  1 0 0 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001000;
	}
	if((544 < adcval) && (adcval < 608))        //  1 0 0 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001001;
	}
	if((608 < adcval) && (adcval < 672))        //  1 0 1 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001010;
	}
	if((672 < adcval) && (adcval < 736))        //  1 0 1 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001011;
	}
	if((736 < adcval) && (adcval < 800))        //  1 1 0 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001100;
	}
	if((800 < adcval) && (adcval < 864))        //  1 1 0 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001101;
	}
	if((864 < adcval) && (adcval < 928))        //  1 1 1 0 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001110;
	}
	if(928 < adcval)			//  1 1 1 1 / 0 - 1023
	{
		SWR &= 0b11110000;
		SWR |= 0b00001111;
	}
}