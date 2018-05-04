/*
 * Einderit.c
 *
 * Created: 27/04/2018 13:09:15
 *  Author: vincent
 */ 

#include "Einderit.h"

void timer_init()
{
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TCNT1 = 0; // set timer counter initial value (16 bit value)
	
	// Interrupts on overflow (every ~1 second)
	OCR1A = 0x002E;
	
	TIMSK1 = 1 << OCIE1A; // enable timer compare match 1A interrupt

	sei(); // enable interrupts
}

ISR(TIMER1_COMPA_vect) // 16 bit timer 1 compare 1A match
{
	//checken of-en welke einderit contacten actief zijn, zoja, overeenkomstigge motor stopzetten.
	//(reminder)check if interrupts will interrupt other functionality (probably will).
}