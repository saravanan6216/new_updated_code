#include "stm32l4xx.h"
void sys_clock_config(void)
{
	//1. ENABLE HSI and wait for the HSI to become Ready

RCC->CR |= RCC_CR_HSION;

while(!(RCC->CR & RCC_CR_HSIRDY));
	
	//2. Configure the PRESCALARS HCLK, PCLK1, PCLK2
	
	//AHB PR
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	
	//APB1 PR
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
	
	//APB2 PR
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
	
	RCC->CFGR &= ~(RCC_CFGR_SW);
	RCC->CFGR |= RCC_CFGR_SW_HSI;
}

void ADC_initialisation(void)
{
	//1.enable ADC and GPIO clock
	RCC->AHB2ENR |= (1<<0); //enable GPIO CLOCK
	RCC->AHB2ENR |= (1<<13); //Enable ADC clock
	
	//
	//2.set the prescalar in the common control register
	ADC1->CR =0;
	
	 // 3.Configure ADC clock (PCLK2 divided by 2)
    ADC123_COMMON->CCR =0x00020000;//10: HCLK/2 (Synchronous clock mode) //CKMODE[1:0]: ADC clock mode
	
	ADC1->CFGR = 0x80000000; //A set or reset of JQDIS bit causes the injected queue to be flushed and the JSQR register is cleared.
	
	ADC1->SQR1|=(1<<6); //ADC regular sequence register 1 
	
}
int main()
{
	
	sys_clock_config();
	ADC_initialisation();

	// Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	
	//DEBUGGING
	//reset the port pin
	GPIOA->MODER &=~((1<<11)|(1<<10));
	
	//Set the pin as output
	GPIOA->MODER |= (1<<10); 

	// Configure PC0 as analog input
    GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1;
	GPIOC->ASCR |= 1;
	while(1)
		{
//DEBUGGING
			GPIOA->BSRR |= (1<<5);
		
		
		
	// Start ADC conversion
        ADC1->CR |= 0x10000005;
	
	while (!(ADC1->ISR & ADC_ISR_EOC)); //ADC interrupt and status register
	/* EOC: End of conversion flag
This bit is set by hardware at the end of each regular conversion of a channel when a new data is 
available in the ADC_DR register. It is cleared by software writing 1 to it or by reading the ADC_DR 
register */
	ADC123_COMMON->CDR; //ADC common regular data register
		}
}

