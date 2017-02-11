#include "Histogram.h"


#define NUM_BUCKETS (101)

void ClearHist(uint16_t* pulse_time_hist) {
    unsigned int i;
    for (i = 0; i < NUM_BUCKETS; i++) {
        pulse_time_hist[i] = 0;
    }
}

void PrintHist(uint16_t* pulse_time_hist, uint8_t* buffer, int lower_limit) {
    unsigned int i;
    int n;
    unsigned int count = 0;
    
    n = sprintf((char *)buffer, "Displaying Histogram: \r\n");
    USART_Write(USART2, buffer, n);
    
    for (i = 0; i < NUM_BUCKETS; i++) {
        count = pulse_time_hist[i];
        
        if (count != 0) {
            n = sprintf((char *)buffer, "Pulse Duration: %uuS : Count: %u\r\n", lower_limit + i,count);
            USART_Write(USART2, buffer, n);
        }
    }
}

void AddValueToHist(uint16_t* pulse_time_hist, int index) {
    // If greater than or less than bounds then truncate to the limit
    if (index < 0) {
        index = 0;
    } else if (index > 100) {
        index = 100;
    }
    pulse_time_hist[index] += 1;
}