#include "GPIOA.h"

void gpioAInit(void) {
	
	    // Enable clk to PortA
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	    // Configure GPIO Pin A0 for alternate function AF1 such that it is routed
    // to TIM2_CH1
    
    // Enable the clock to GPIO Ports A	
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    // Set PA0 to be alternate function
    GPIOA->MODER &= ~GPIO_MODER_MODER0;	    // Clear moder register mode0[1:0]
    GPIOA->MODER |= GPIO_MODER_MODER0_1;    // Set alternate function mode 10
    // Set Alternate function lower register to AF1 so that A1 is set connected to TIM2_CH1
    GPIOA->AFR[0] = 0x1;
}
