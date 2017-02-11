#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#define NUM_BUCKETS (101)
#define NUM_MEASUREMENTS (1000)
#define MIN_LOWER_LIMIT (50)
#define MAX_LOWER_LIMIT (9950)
#define DEFAULT_LOWER_LIMIT (950)

unsigned int wasPost = 0;
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
	int n;
	
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
    int n;
    uint16_t which_interrupt = TIM2->SR;
    which_interrupt &= 3;
	  if (which_interrupt == 0x202) {
			Green_LED_On();Red_LED_On();}
	
    // Check for overflow interrupt
    if (((which_interrupt & 1 ) == 1)/* && (state == STATE_POST))*/) {
        Red_LED_Toggle();
		processEvent(EVENT_POST_COMPLETE);
		TIM2->CCER &= ~TIM_CCER_CC1E;
		TIM2->DIER &= ~3;
		TIM2->CR1 &= ~1;
        TIM2->SR &= ~1; // Clear overflow interrupt
    }
    
    // Input channel 1 capture interrupt
    if ((which_interrupt & 2) == 2) {
        current_rising_edge_count = TIM2->CCR1;
        
			  processEvent(EVENT_RISING_EDGE_DETECT);
        
        //last_rising_edge_count = current_rising_edge_count;
        //Green_LED_Toggle();
		    
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
                return DEFAULT_LOWER_LIMIT;
            }
            break;
        }
        // Echo the received character back to the display
        n = sprintf((char *)buffer, "%c",rx_arr[length]);
        USART_Write(USART2, buffer, n);
		// Ascii code for 0-9 is 30 for 0 -> 39 for 9, hence the bottom nibble contains the int 
		// Therefore mask with 0xF to capture just the bottom nibble
		rx_arr[length] &= 0xF;
        length += 1;
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
	int		n ;
    
	System_Clock_Init();
    
    // Enable clk to PortA
    #warning MOVE oneoff statements into an init function
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    
	LED_Init();
	UART2_Init();
    
  // Configure GPIO Pin
    // Enable the clock to GPIO Ports A	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Set PA1 to be alternate function
    GPIOA->MODER &= ~3 ;	
    GPIOA->MODER |= 0x2 ; // Set PA1 to alternate function mode 
    // Set Alternate function lower register to AF1 so that A1 is set connectted to TIM2_CH2
    GPIOA->AFR[0] = 0x1;
    
    // Configure Timer 2 Channel 2 For Input Capture
	
    // Enable clock of timer 2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    
    // Enable interrupt for updating timer registers
    //TIM2->CR1 |= 2;     // Set UDIS bit
    
		TIM2->CR1 |= 4;
		
    // Set Prescaler
    // 80 MHz / 80 = 1 MHz -> 1 us
    TIM2->PSC = 79;
    // Configure active input for  Capture/Compare 
    // Configure CC1 as an input and map IC1 to TI2
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= 0x1; 
    TIM2->EGR |= 1;
    // Select the edge of the capture
    // Enable Capture from the counter
    TIM2->CCER &= ~0xa; 
    
	
		NVIC_EnableIRQ(TIM2_IRQn);     
	
    // Kick of a state machine update event
  rising_edge_count = 0;
	update_SM = 1;
	state = STATE_POST;
	
	while (1){
        if (update_SM == 1) { 
            switch (state) {
                case (STATE_POST):
                    update_SM = 0;
                    
					
                    //Red_LED_On();
                    //Green_LED_Off();
                    n = sprintf((char *)buffer, "Running POST!\r\n");
                    USART_Write(USART2, buffer, n);
                    wasPost = 1;  
										TIM2->PSC = 3999;
								    TIM2->ARR = 20000;
										TIM2->EGR |= 1;
								    TIM2->DIER |= 3;
								    TIM2->CCER |= TIM_CCER_CC1E;
								    TIM2->CR1 |= 1;
                    break;
                    
                case (STATE_POST_FAIL_PROMPT):
                    //Red_LED_Off();
                    //Green_LED_Off();
                    n = sprintf((char *)buffer, "POST Failed\r\n");
                    n += sprintf((char *)buffer+n, "Press ENTER to re-run POST...\r\n");
                    USART_Write(USART2, buffer, n);	
                    while (USART_Read(USART2) != 0x0d);
                    processEvent(EVENT_RERUN_POST);
                    break;
                    
                case (STATE_PARSE_LIMITS):
				

										Green_LED_Off();Red_LED_Off();
                    //Red_LED_Off();
                    //Green_LED_On();
                    if (wasPost == 1) {
                        n = sprintf((char *)buffer, "Post PASSED!\r\n");
                        USART_Write(USART2, buffer, n);	
                    }
                    n = sprintf((char *)buffer, "%u edges detected.\r\n", rising_edge_count);
                    // remove?
					rising_edge_count = 0;
                    USART_Write(USART2, buffer, n);	
					lower_limit = 0;
                    while(validLowerLimit(lower_limit) == 0) {
                        n = sprintf((char *)buffer, "Enter lower limit in range of 50-9950 microseconds and press ENTER: ");
                        USART_Write(USART2, buffer, n);	
                        lower_limit = parseLowerLimit();
                    }
                    n = sprintf((char *)buffer, "Starting with lower limit %d\r\n", lower_limit);
                    USART_Write(USART2, buffer, n);	
                    processEvent(EVENT_START_MEASUREMENTS);
                    break;
                    
                case (STATE_PERFORM_MEASUREMENTS):
                    //Red_LED_Off();
                    //Green_LED_Off();
                    clearHist();
                    n = sprintf((char *)buffer, "1000 Measurements in progress...\r\n");
                    USART_Write(USART2, buffer, n);	
                    update_SM = 0;
								    TIM2->ARR = 0xffff;
								    TIM2->PSC = 79;
								    TIM2->EGR = 1;
								    TIM2->DIER = 0x2;//TIM_CCER_CC1E;
								TIM2->CCER |= TIM_CCER_CC1E;    
								TIM2->CR1 |= TIM_CR1_CEN;   // Enable CEN bit
								    
								    
                    break;
                    
                case (STATE_DISPLAY_HIST):
									  TIM2->DIER &= ~0x2;//TIM_CCER_CC1E;
                    TIM2->CR1 &= ~TIM_CR1_CEN;   // Enable CEN bit
								    TIM2->CCER &= ~TIM_CCER_CC1E;
                    printHist();
                    processEvent(EVENT_HIST_DISP_DONE);
                    wasPost = 0;
                    break;
                    
                case (STATE_FAULT):    
                default:
                    //Red_LED_On();
                    //Green_LED_On();
                    n = sprintf((char *)buffer, "FAULT STATE\r\n");
                    USART_Write(USART2, buffer, n);	
                    update_SM = 0;
                
            } // end case
        }
    }
}
