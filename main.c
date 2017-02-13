/******************************************************************************
* FILENAME : main.c          
*
* DESCRIPTION : 
*     Perform 1000 rising edge period measurements and display histogram of results
*     to the terminal
*
* AUTHOR: 
*     Donald MacIntyre - djm4912@rit.edu
*     Madison Smith    - ms8565@rit.edu  
*
******************************************************************************/

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "histogram.h"
#include "timer2.h"
#include "GPIOA.h"

#define debug (0)
#define NUM_MEASUREMENTS (1000)
#define MIN_LOWER_LIMIT (50)
#define MAX_LOWER_LIMIT (9950)
#define DEFAULT_LOWER_LIMIT (950)

/******************************************************************************
* GLOBAL VARIABLES
******************************************************************************/
unsigned int lower_limit = 0;
uint16_t last_rising_edge_count = 0;
uint16_t current_rising_edge_count = 0;
uint16_t delta_time = 0;
uint8_t buffer[BufferSize];
unsigned int rising_edge_count = 0;
unsigned int update_SM = 0;

typedef enum {
    STATE_POST,
	STATE_POST_FAIL_PROMPT,
	STATE_PARSE_LIMITS,
	STATE_PERFORM_MEASUREMENTS,
	STATE_DISPLAY_HIST,
    STATE_FAULT,
} state_t;

state_t state = STATE_POST;

// When post complete determine if next state based on number of edges seen during POST
void processPOSTComplete(){
    if (rising_edge_count == 0) {
        state = STATE_POST_FAIL_PROMPT;
    } else {
        state = STATE_PARSE_LIMITS;
    }
    // Force the main loop state machine to update
    update_SM = 1;
}

// When a rising edge is detected parse results based current state	
void processRisingEdge(){
	int index;
	
	switch (state) {
        case (STATE_POST):
            rising_edge_count += 1;
            break;
        case (STATE_PERFORM_MEASUREMENTS):
		    // If not first time, then find delta time
			if (rising_edge_count != 0) {
                // If overflow occurred
                if (current_rising_edge_count < last_rising_edge_count) {
                    // Overflow occurred, delta time = (2^16 - last_rising_edge_count) + current_rising_edge_count
					delta_time = (0xFFFF - last_rising_edge_count) + current_rising_edge_count; 
					// No overflow occurred, calculate delta time normally
				} else {
                    delta_time = current_rising_edge_count - last_rising_edge_count;
				}
						
				// Calculate index into histogram array
				// If greater than or less than bounds then truncate to the limit
				index = delta_time - lower_limit;
				addValueToHist(index);
			} 
					
            // If its the last rising edge then kick state machine to display histogram
			if (rising_edge_count == (NUM_MEASUREMENTS)) {
                state = STATE_DISPLAY_HIST;
                update_SM = 1;
				// Still have more edges to measure
            } else {
                rising_edge_count += 1;
				last_rising_edge_count = current_rising_edge_count;
			}		
            break;
		default:
			update_SM = 1;
			state = STATE_FAULT;
	}
}

/******************************************************************************
* INTERRUPT HANDLERS
******************************************************************************/

void TIM2_IRQHandler(void) {
    uint16_t which_interrupt = TIM2->SR;
    which_interrupt &= (TIM_SR_UIF | TIM_SR_CC1IF);   // Only look at the channel 1 capture and overflow status flags 
	
    // Check for overflow interrupt
    if (((which_interrupt & TIM_SR_UIF) == TIM_SR_UIF)) {
        processPOSTComplete();
		timer2DisableInterrupts();
    }
    
    // Input channel 1 capture interrupt
    if ((which_interrupt & TIM_SR_CC1IF) == TIM_SR_CC1IF) {
        // Read the latched time from the capture event
        // this also clears the capture flag in TIM2->SR
        current_rising_edge_count = TIM2->CCR1;
        processRisingEdge();
    }
}

// Parse the lower limit for custom user input
int parseLowerLimit() {
    uint8_t rx_arr[10];
    unsigned int value = 0;
    unsigned int length = 0;
    int n;
	unsigned int i;
	
    while (1) {
        rx_arr[length] = USART_Read(USART2);
        if ((rx_arr[length]) == 0x0d) {
			// User hits ENTER without entering any characters so select default Lower Limit
            if (length == 0) {
                // Write a new line after echoing user input
                n = sprintf((char *)buffer, "\r\n");
                USART_Write(USART2, buffer, n);
                return 0;
            }
            break;
        }
        if ((rx_arr[length] >= 0x30) && (rx_arr[length] <= 0x39)) {
            // Echo the received character back to the display
            n = sprintf((char *)buffer, "%c",rx_arr[length]);
            USART_Write(USART2, buffer, n);
            // Ascii code for 0-9 is 30 for 0 -> 39 for 9, hence the bottom nibble contains the int 
            // Therefore mask with 0xF to capture just the bottom nibble
            rx_arr[length] &= 0xF;
            length += 1;
        }
    }
	for (i = 0; i < length; i++) {
		value += rx_arr[i] * pow(10,length-i-1);
	}
    // Write a new line after echoing user input
    n = sprintf((char *)buffer, "\r\n");
    USART_Write(USART2, buffer, n);
	
    return value;
}

// Check if a given lower limit is valid
int validLowerLimit(int limit) {
    if ((limit >= MIN_LOWER_LIMIT) && (limit <= MAX_LOWER_LIMIT)) {
        return 1;
    } else {
        return 0;
    }
}

int main(void){
	int n;
    unsigned int wasPost;
    char rxbyte;
   
	System_Clock_Init();

#if debug
	LED_Init();
#endif

	UART2_Init();
    gpioAInit();
    
    // Configure Timer 2 Channel 1 For Input Capture
	  timer2InitialConfig();
     
    
	// Enable the interrupt handler
	NVIC_EnableIRQ(TIM2_IRQn);     
	
    // Intialize the state machine
    rising_edge_count = 0;
	update_SM = 1;
	state = STATE_POST;
	
    // State Machine Main loop
	while (1){
        // Pend on state machine update event which is posted by interrupt handler
        // update_SM is flag that is set by interrupt handler on state transitions
        if (update_SM == 1) { 
            switch (state) {
                case (STATE_POST):
                    update_SM = 0;
#if debug
                    Red_LED_On();
                    Green_LED_Off();
#endif
                    n = sprintf((char *)buffer, "Running POST!\r\n");
                    USART_Write(USART2, buffer, n);
                    wasPost = 1;    // Indicate POST was just run so STATE_PARSE_LIMITS knows if it should print POST PASS status
                    timer2PostInit();
                    
                    break;
                    
                case (STATE_POST_FAIL_PROMPT):
#if debug                    
                    Red_LED_Off();
                    Green_LED_Off();
#endif                    
                    n = sprintf((char *)buffer, "POST Failed\r\n");
                    n += sprintf((char *)buffer+n, "Press ENTER to re-run POST...\r\n");
                    USART_Write(USART2, buffer, n);	
                    // Wait for the user to enter a carrige return to re-run POST
                    while (USART_Read(USART2) != 0x0d);
                    state = STATE_POST;
                    update_SM = 1;
                    break;
                    
                case (STATE_PARSE_LIMITS):
#if debug				
                    Red_LED_Off();
                    Green_LED_On();
#endif
                    // Only print status of POST if we just ran it 
                    if (wasPost == 1) {
                        n = sprintf((char *)buffer, "Post PASSED!\r\n");
                        n += sprintf((char *)buffer + n, "%u edges detected.\r\n", rising_edge_count);
                        USART_Write(USART2, buffer, n);	
                    }
                    
                    // Set lower limit to invalid and loop until user enters a valid limit
					n = sprintf((char *)buffer, "Current lower limit is: %uus\r\n",DEFAULT_LOWER_LIMIT);
                    n += sprintf((char *)buffer + n, "Current upper limit is: %uus\r\n",DEFAULT_LOWER_LIMIT + 100);
                    n += sprintf((char *)buffer + n, "Accept? [y/n]: ");
                    USART_Write(USART2, buffer, n);	
                    rxbyte = USART_Read(USART2);
                    // Echo user char back to terminal
                    n = sprintf((char *)buffer, "%c\r\n",rxbyte);
                    USART_Write(USART2, buffer, n);
                    
                    // User has selected yes to change the limit
                    if ((rxbyte == 'N') || (rxbyte == 'n')) {
                        lower_limit = 0;    // set limit to invalid to force user to enter 
                        while(validLowerLimit(lower_limit) == 0) {
                            n = sprintf((char *)buffer, "Enter lower limit in range of 50-9950 microseconds and press ENTER: ");
                            USART_Write(USART2, buffer, n);	
                            lower_limit = parseLowerLimit();
                        }    
                    // User wants default lower limit
                    } else {
                        lower_limit = DEFAULT_LOWER_LIMIT;
                    }  
                    // Output limits to run measurements with
                    n = sprintf((char *)buffer, "Starting with lower limit %dus\r\n", lower_limit);
                    n += sprintf((char *)buffer + n, "Starting with upper limit %dus\r\n", lower_limit + 100);
                    USART_Write(USART2, buffer, n);	
                    update_SM = 1;
                    rising_edge_count = 0;
                    state = STATE_PERFORM_MEASUREMENTS;
                    break;
                    
                case (STATE_PERFORM_MEASUREMENTS):
#if debug               
                    Red_LED_Off();
                    Green_LED_Off();
#endif                     
                    clearHist();
                    n = sprintf((char *)buffer, "1000 Measurements in progress...\r\n");
                    USART_Write(USART2, buffer, n);	
                    update_SM = 0;
                    
                    timer2MeasurementInit();
								    
								    
                    break;
                    
                case (STATE_DISPLAY_HIST):
                    timer2DisableInterrupts();
                    printHist(buffer, lower_limit);
                    update_SM = 1;
                    state = STATE_PARSE_LIMITS;
                    wasPost = 0;
                    break;
                    
                case (STATE_FAULT):    
                default:
#if debug
                    Red_LED_On();
                    Green_LED_On();
                    n = sprintf((char *)buffer, "FAULT STATE\r\n");
                    USART_Write(USART2, buffer, n);	
#endif 
                    update_SM = 0;
                
            } // end case
        }
    }
}
