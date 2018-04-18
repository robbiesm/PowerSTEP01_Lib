/*
 * PowerSTEP_R2R.h
 *
 * Created: 17/04/2018 13:08:04
 *  Author: vincent
 */ 


#ifndef POWERSTEP_R2R_H_
#define POWERSTEP_R2R_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define SWR GPIOR0		//switch register
#define SW1 GPIOR00
#define SW2 GPIOR01
#define SW3 GPIOR02
#define SW4 GPIOR03

void ADCInit(void);
void getADC(void);

#endif /* POWERSTEP_R2R_H_ */

