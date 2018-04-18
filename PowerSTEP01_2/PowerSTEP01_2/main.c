/*
 * Powerstep01.c
 *
 * Created: 26/03/2018 12:36:56
 * Author : robbi
 */ 

#include <avr/io.h>
#include "Library/SPI/SPI.h"
#include "Library/ADC/R2R.h"
#include "Library/PWM/PWM.h"
#include "Library/powerSTEP01/powerSTEP01.h"

int main(void)
{
	SPI_Init();
	motorControl_Init();
	PWMinit();
	ADCInit();
    /* Replace with your application code */
    while (1) 
    {
		//PWM_DutyCycle = (uint8_t)(getADC() >> 2);
    }
}

