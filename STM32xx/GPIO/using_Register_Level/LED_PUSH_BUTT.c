#include "stm32l4xx.h"                  // Device header
int main()
{
  RCC->AHB1ENR =(1<<0)|(1<<2);
	GPIOA->MODER =(1<<10);
	GPIOC->MODER =(0<<26);
	while(1)
	{
		if(GPIOC->IDR & 1<<13)
		{
			GPIOA->BSRR=(1<<21);
		}
		else
		{
			GPIOA->BSRR=(1<<5);
		}
	}
}
