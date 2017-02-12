#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#define debug (1)
#define NUM_BUCKETS (101)
#define NUM_MEASUREMENTS (1000)
#define MIN_LOWER_LIMIT (50)
#define MAX_LOWER_LIMIT (9950)
#define DEFAULT_LOWER_LIMIT (950)

unsigned int lower_limit = 0;
uint16_t last_rising_edge_count = 0;
uint16_t current_rising_edge_count = 0;
uint16_t delta_time = 0;
uint8_t buffer[BufferSize];
unsigned int rising_edge_count = 0;
unsigned int update_SM = 0;
// Initalize this to 1 as when initalizing to 0 and clearing some elements are non-zero
// Possibly a compiler bug??? Changing optimatization level from -o0 to default elmiinates the issue
uint16_t pulse_time_hist[NUM_BUCKETS] = {1};

typedef enum {
    STATE_POST,
	STATE_POST_FAIL_PROMPT,
	STATE_PARSE_LIMITS,
	STATE_PERFORM_MEASUREMENTS,
	STATE_DISPLAY_HIST,
    STATE_FAULT,
} state_t;

typedef enum {
    EVENT_POST_COMPLETE,
    EVENT_RISING_EDGE_DETECT,
    EVENT_RERUN_POST,
    EVENT_START_MEASUREMENTS,
    EVENT_HIST_DISP_DONE,
    EVENT_MAX_EVENT
} event_t;

state_t state = STATE_POST;

void processEvent(event_t event) {
	
	int index = 0;
	
    switch(event) {
        // 100 ms post time is over
        case (EVENT_POST_COMPLETE):
            switch (state) {
                case(STATE_POST):
                    if (rising_edge_count == 0) {
                        state = STATE_POST_FAIL_PROMPT;
                    } else {
                        state = STATE_PARSE_LIMITS;
                    }
                    update_SM = 1;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break;  // end EVENT_POST_COMPLETE
        
        case (EVENT_RISING_EDGE_DETECT):
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
						if (index < 0) {
							index = 0;
						} else if (index > 100) {
							index = 100;
						}
						pulse_time_hist[index] += 1;
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
            } // end state
            break;  // end EVENT_POST_COMPLETE
        
        case (EVENT_RERUN_POST):
            switch (state) {
                case(STATE_POST_FAIL_PROMPT):
                    state = STATE_POST;
                    update_SM = 1;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break; // end EVENT_RERUN_POST
        
        case (EVENT_START_MEASUREMENTS):
            switch (state) {
                case (STATE_PARSE_LIMITS) :
                    rising_edge_count = 0;
                    update_SM = 1;
                    rising_edge_count = 0;
                    state = STATE_PERFORM_MEASUREMENTS;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break;  // end EVENT_START_MEASUREMENTS
            
        case (EVENT_HIST_DISP_DONE):
            switch(state) {
                case (STATE_DISPLAY_HIST):
                    update_SM = 1;
                    state = STATE_PARSE_LIMITS;
                    break;
                default:
                    update_SM = 1;
                    state = STATE_FAULT;
            } // end state
            break; // end EVENT_HIST_DISP_DONE
            
        default:
            update_SM = 1;
            state = STATE_FAULT;
    }
}

void TIM2_IRQHandler(void) {
    uint16_t which_interrupt = TIM2->SR;
    which_interrupt &= 3;   // Only look at the channel 1 capture and overflow status flags 
	
    // Check for overflow interrupt
    if (((which_interrupt & TIM_SR_UIF) == TIM_SR_UIF)) {
        processEvent(EVENT_POST_COMPLETE);
        // Disable capture on channel 1
		TIM2->CCER &= ~TIM_CCER_CC1E;
        // Disable interrupts for both UIF and channel 1 capture
		TIM2->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_UIE);
        // Disable TIM2
		TIM2->CR1 &= ~TIM_CR1_CEN;
        TIM2->SR &= ~TIM_SR_UIF; // Clear overflow interrupt
    }
    
    // Input channel 1 capture interrupt
    if ((which_interrupt & TIM_SR_CC1IF) == TIM_SR_CC1IF) {
        // Read the latched time from the capture event
        // this also clears the capture flag in TIM2->SR
        current_rising_edge_count = TIM2->CCR1;
        processEvent(EVENT_RISING_EDGE_DETECT);
    }
}


void clearHist() {
    unsigned int i;
    for (i = 0; i < NUM_BUCKETS; i++) {
        pulse_time_hist[i] = 0;
    }
}

void printHist() {
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
    
    // Enable clk to PortA
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

#if debug
	LED_Init();
#endif

	UART2_Init();
    
    // Configure GPIO Pin A0 for alternate function AF1 such that it is routed
    // to TIM2_CH1
    
    // Enable the clock to GPIO Ports A	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Set PA0 to be alternate function
    GPIOA->MODER &= ~GPIO_MODER_MODER0;	    // Clear moder register mode0[1:0]
    GPIOA->MODER |= GPIO_MODER_MODER0_1;    // Set alternate function mode 10
    // Set Alternate function lower register to AF1 so that A1 is set connected to TIM2_CH1
    GPIOA->AFR[0] = 0x1;
    
    // Configure Timer 2 Channel 1 For Input Capture
	
    // Enable clock of timer 2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    
    // Only allow an update event (UIF) when an overflow occurs 
	TIM2->CR1 |= TIM_CR1_URS;
		
    // Set Prescaler
    // 80 MHz / 80 = 1 MHz -> 1 us
    TIM2->PSC = 79;
    // Configure active input for  Capture/Compare 
    // Configure CC1 as an input and map IC1 to TI1
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; 
    #warning THIS ISNT NEEDED??? DONE IN STATE POST
    //TIM2->EGR |= TIM_EGR_UG; // Re-initalize the counter and generate an update of the registers
    // Select rising edge for capture
    TIM2->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P); 
    
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
                    // Set Prescaler To count at 20 KHz -> 80 MHz / 4000 = 20 kHz -> 50 us
					TIM2->PSC = 3999;
                    // Set Auto Reload Register to 2000 to get an overflow interrupt to end POST after 100 ms -> 2000 * 50us = 100 ms
                    TIM2->ARR = 2000;
                    TIM2->EGR |= TIM_EGR_UG;
                    // Unmask TIM2 Interrupts
                    // UIF - Overflow occurs -> POST over
                    // Channel 1 Input Capture -> rising edge occurred
					TIM2->DIER |= (TIM_DIER_CC1IE | TIM_DIER_UIE);
                    // Enable capture on Channel 1
					TIM2->CCER |= TIM_CCER_CC1E;
                    // Enable TIM2 -> Important to do this last otherwise we will get a UIF in the TIM2->SR 
                    // which we will incorrectly process as an overflow event 
					TIM2->CR1 |= TIM_CR1_CEN;
                    
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
                    processEvent(EVENT_RERUN_POST);
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
                    processEvent(EVENT_START_MEASUREMENTS);
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
                    // Set UDIS bit to disable UIF flag
					//TIM2->CR1 |= TIM_CR1_UDIS;
                    // Set the Auto Reload Register back to full scale to avoid overflows
					TIM2->ARR = 0xffff;
                    // Set Prescaler to count at 1 MHz -> 80 MHz / 80 = 1 MHz -> 1us
					TIM2->PSC = 79;
					TIM2->EGR |= TIM_EGR_UG;
					TIM2->CR1 |= TIM_CR1_UDIS;
                    // Enable TIM2_CH1 capture interrupt
					TIM2->DIER = TIM_DIER_CC1IE;
                    // Enable capture on channel 1
                    TIM2->CCER |= TIM_CCER_CC1E;  
                    // Enable TIM2
                    TIM2->CR1 |= TIM_CR1_CEN; 
								    
								    
                    break;
                    
                case (STATE_DISPLAY_HIST):
                    // Disable capture interrupts
					TIM2->DIER &= ~TIM_DIER_CC1IE;
                    // Disable TIM2
                    TIM2->CR1 &= ~TIM_CR1_CEN; 
                    // Disable channel 1 capture                    
					TIM2->CCER &= ~TIM_CCER_CC1E;
                    printHist();
                    processEvent(EVENT_HIST_DISP_DONE);
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
