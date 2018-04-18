/*
 * Powerstep01.c
 *
 * Created: 26/03/2018 12:36:56
 * Author : robbi
 */ 

#include <avr/io.h>
#include "Library/SPI/SPI.h"
#include "Library/ADC/PowerSTEP_R2R.h"
#include "Library/PWM/PowerSTEP_PWM.h"
#include "Library/powerSTEP01/powerSTEP01.h"

int main(void)
{
	//SPI_Init();
	//motorControl_Init();

    /* Replace with your application code */
    while (1) 
    {
		/*move(0, 1, 12800);
		move(0, 0, 25600);
		move(0, 1, 12800);
		move(0, 0, 6400);
		move(0, 1, 6400);
		move(0, 0, 12800);
		move(0, 0, 64000);
		run(0, 0, 100);
		_delay_ms(20000);
		softStop(0);
		_delay_ms(1000);*/
		getADC();
    }
}

