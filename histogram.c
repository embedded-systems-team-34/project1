/******************************************************************************
* FILENAME : histogram.c          
*
* DESCRIPTION : 
*     Histogram is used to store edge period data
*
* AUTHOR: 
*     Donald MacIntyre - djm4912@rit.edu
*     Madison Smith    - ms8565@rit.edu  
*
******************************************************************************/
#include "histogram.h"

// Initalize this to 1 as when initalizing to 0 and clearing some elements are non-zero
// Possibly a compiler bug??? Changing optimatization level from -o0 to default elmiinates the issue
uint16_t pulse_time_hist[NUM_BUCKETS] = {1};

// Resets all values stored in the histogram
void clearHist() {
    unsigned int i;
    for (i = 0; i < NUM_BUCKETS; i++) {
        pulse_time_hist[i] = 0;
    }
}

// Print the histogram out to the Serial Port
void printHist(uint8_t* buffer, int lower_limit) {
    unsigned int i;
    int n;
    unsigned int count = 0;
    
	  n = sprintf((char *)buffer, "**********************************\r\n");
    n += sprintf((char *)buffer+n, "Displaying Histogram: \r\n");
	  n += sprintf((char *)buffer+n, "**********************************\r\n");
    USART_Write(USART2, buffer, n);
    for (i = 0; i < NUM_BUCKETS; i++) {
        count = pulse_time_hist[i];
        if (count != 0) {
            n = sprintf((char *)buffer, "Pulse Duration: %uuS : Count: %u\r\n", lower_limit + i,count);
            USART_Write(USART2, buffer, n);
        }
    }
		n = sprintf((char *)buffer, "**********************************\r\n");
    USART_Write(USART2, buffer, n);
}

// Add a value into the histogram
void addValueToHist(int index) {
    // If greater than or less than bounds then truncate to the limit
    if (index < 0) {
        index = 0;
    } else if (index > 100) {
        index = 100;
    }
    pulse_time_hist[index] += 1;
}
