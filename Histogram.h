#ifndef HISTOGRAM_FUNCTIONS
#define HISTOGRAM_FUNCTIONS 

#include "stm32l476xx.h"
#include <string.h>
#include <stdio.h>
#include "UART.h"

void ClearHist(uint16_t* pulse_time_hist);

void PrintHist(uint16_t* pulse_time_hist, uint8_t* buffer, int lower_limit);

void AddValueToHist(uint16_t* pulse_time_hist, int index);



#endif

