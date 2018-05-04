/*
 * Einderit.h
 *
 * Created: 27/04/2018 13:09:43
 *  Author: vincent
 */ 


#ifndef EINDERIT_H_
#define EINDERIT_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
//#include <stdlib.h>
#include <avr/interrupt.h>

void timer_init(void);

#endif /* EINDERIT_H_ */