/******************************************************************************
* FILENAME : histogram.h          
*
* DESCRIPTION : 
*     Function prototypes for histogram 
*
* AUTHOR: 
*     Donald MacIntyre - djm4912@rit.edu
*     Madison Smith    - ms8565@rit.edu  
*
******************************************************************************/
#ifndef HISTOGRAM_FUNCTIONS
#define HISTOGRAM_FUNCTIONS 

#include <stdio.h>
#include "UART.h"

#define NUM_BUCKETS (101)

// Function Prototypes
void clearHist(void);
void printHist(uint8_t* buffer, int lower_limit);
void addValueToHist(int index);

#endif
