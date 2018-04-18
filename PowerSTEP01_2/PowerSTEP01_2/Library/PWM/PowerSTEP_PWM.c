/*
 * PowerSTEP_PWM.c
 *
 * Created: 17/04/2018 14:16:51
 *  Author: vincent
 */ 

#include "PowerSTEP_PWM.h"

void PWMinit(void)
{
	DDRD |= (1<<PORTD6);
	TCCR0A |= (1<<COM0A1)|(1<<WGM00)|(1<<WGM01);
	TCCR0B |= (1<<CS00);
}