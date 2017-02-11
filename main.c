#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"

#include <string.h>
#include <stdio.h>

char RxComByte = 0;
uint8_t buffer[BufferSize];
char str[] = "Give Red LED control input (Y = On, N = off):\r\n";

void TIM2_IRQHandler(void) {
    
    // Check for overflow interrupt
    if ((TIM2->SR & 1 ) == 1) {
        Red_LED_Toggle();
        TIM2->SR &= ~1; // Clear overflow interrupt
    }
    
    // Input channel 1 capture interrupt
    if ((TIM2->SR & 2) == 2) {
        uint16_t count = TIM2->CCR1;
        Green_LED_Toggle();
    }
}

int main(void){
	//char rxByte;
	int		a ;
	int		n ;
	int		i ;
	float b;
	
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	//UART2_Init();
    
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
    
    // Set Prescaler
    // 80 MHz / 80 = 1 MHz -> 1 us
    TIM2->PSC = 79;
    // Configure active input for  Capture/Compare 
    // Configure CC1 as an input and map IC1 to TI2
    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 |= 0x1; 
    // Select the edge of the capture
    // Enable Capture from the counter
    TIM2->CCER &= ~0xa; 
    TIM2->CCER |= TIM_CCER_CC1E;
    
    // Enable the Timer2 capture interrupt for CC2IE
    TIM2->DIER |= 0x2;//TIM_CCER_CC1E;
    // Enable interrupt for a overflow event
    TIM2->DIER |= 1;
    TIM2->ARR = 1000;
    TIM2->EGR |= 1;
    
    // Enable the Timer2 counter
    TIM2->CR1 |= TIM_CR1_CEN;   // Enable CEN bit
    //TIM2->EGR = 2;
    // Enable TIM2 Interrupt
    NVIC_EnableIRQ(TIM2_IRQn);    
	
	while (1){
     //if (TIM2->CCR1 != 0) {
	//		 Green_LED_On();
	//	 }
        
	}
}
