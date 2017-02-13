/******************************************************************************
* FILENAME : timer2.h          
*
* DESCRIPTION : 
*     Function prototypes for timer2 
*
* AUTHOR: 
*     Donald MacIntyre - djm4912@rit.edu
*     Madison Smith    - ms8565@rit.edu  
*
******************************************************************************/

#ifndef TIMER_H
#define TIMER_H 

#include "stm32l476xx.h"

// Function Prototypes
// Set up timers after a reset
void timer2InitialConfig(void);
// Set up timers prior to POST
void timer2PostInit(void);
// Disable timer interrupts and capture
void timer2DisableInterrupts(void);
// set up timers prior to Measurements
void timer2MeasurementInit(void);

#endif
