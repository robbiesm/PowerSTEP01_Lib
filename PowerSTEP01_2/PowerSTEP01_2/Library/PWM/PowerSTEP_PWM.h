/*
 * PowerSTEP_PWM.h
 *
 * Created: 17/04/2018 14:17:18
 *  Author: vincent
 */ 


#ifndef POWERSTEP_PWM_H_
#define POWERSTEP_PWM_H_

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>


#define PWM_DutyCycle OCR0 //0 - 255


#endif /* POWERSTEP_PWM_H_ */