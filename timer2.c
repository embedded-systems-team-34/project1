#include "timer2.h"

// Set up timers after a reset
void timer2InitialConfig(void) {
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
    //TIM2->EGR |= TIM_EGR_UG; // Re-initalize the counter and generate an update of the registers
    // Select rising edge for capture
    TIM2->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P);
}
// Set up timers prior to POST
void timer2PostInit(void) {
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
	
}
// set up timers prior to Measurements
void timer2MeasurementInit(void) {
	                    // Set the Auto Reload Register back to full scale to avoid overflows
					TIM2->ARR = 0xffff;
                    // Set Prescaler to count at 1 MHz -> 80 MHz / 80 = 1 MHz -> 1us
					TIM2->PSC = 79;
					TIM2->EGR |= TIM_EGR_UG;
					// Set UDIS bit to disable UIF flag
					TIM2->CR1 |= TIM_CR1_UDIS;
                    // Enable TIM2_CH1 capture interrupt
					TIM2->DIER = TIM_DIER_CC1IE;
                    // Enable capture on channel 1
                    TIM2->CCER |= TIM_CCER_CC1E;  
                    // Enable TIM2
                    TIM2->CR1 |= TIM_CR1_CEN; 
}

// Disable timer interrupts and capture
void timer2DisableInterrupts(void) {
	                    // Disable capture interrupts
					TIM2->DIER &= ~TIM_DIER_CC1IE;
                    // Disable TIM2
                    TIM2->CR1 &= ~TIM_CR1_CEN; 
                    // Disable channel 1 capture                    
					TIM2->CCER &= ~TIM_CCER_CC1E;
	TIM2->SR &= ~TIM_SR_UIF; // Clear overflow interrupt
}
