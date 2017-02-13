#ifndef TIMER_FUNCTIONS
#define TIMER_FUNCTIONS 

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
