#include "stm32l4xx.h"                  // Device header
/*void RCC_EN()
{
	RCC->CR |=0x6b;
	
	
}*/
void gpio_init()
{
	RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN|RCC_AHB2ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	GPIOA->MODER &= ~((1<<10)|(1<<11));
	GPIOC->MODER &= ~((1<<26)|(1<<27));
	
	GPIOA->MODER |= (1<<10);
	GPIOC->MODER = 0xF3FFFFFF;
	//GPIOC->PUPDR |=  (1<<27);
}
void interrupt_en()
{
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; 
	//EXTI->IMR1 &=~(0xFF820000);
	EXTI->IMR1 |=EXTI_IMR1_IM13;//not mask
	EXTI->RTSR1 |= EXTI_RTSR1_RT13;//raising edge en
	EXTI->FTSR1 &= ~EXTI_FTSR1_FT13;//falling edge dis
	NVIC_SetPriority(EXTI15_10_IRQn,1);//priority setting for nvic
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void EXTI15_10_IRQHandler()
{
	/*if((GPIOA->ODR & (1<<5))==1)
		{
	GPIOA->ODR &=~(1<<5);
		}
	else if ((GPIOA->ODR & (1<<5))==0){
		GPIOA->ODR |=(1<<5);
		
		
	}*/
	GPIOA->ODR ^=(1<<5);
	if (EXTI->PR1 & (1<<13))    // If the PA1 triggered the interrupt
	{
		
		EXTI->PR1 |= (1<<13);  // Clear the interrupt flag by writing a 1 
	}EXTI->PR1 |= (1<<13);
}
int main()
{
	// RCC_EN();
	gpio_init();
	
	interrupt_en();
	
	
	
	
	while(1)
	{
		//GPIOA->ODR &=~(1<<5);
	}
	
	
}