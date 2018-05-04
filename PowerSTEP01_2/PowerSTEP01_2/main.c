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
#include "Library/Einderit/Einderit.h"
#include "Library/powerSTEP01/powerSTEP01.h"

int main(void)
{
	timer_init();
	SPI_Init();
	motorControl_Init(1);
	PWMinit();
	ADCInit();
    /* Replace with your application code */
	//setParam(1, 0, STEP_MODE, 0b00000100);
	//getParam(1, STEP_MODE);
	
    while (1) 
    {
		//PWM_DutyCycle = (uint8_t)(getADC() >> 2);
		move(1, 1, 12800);
		move(1, 0, 12800);
		_delay_ms(2500);
    }
}

