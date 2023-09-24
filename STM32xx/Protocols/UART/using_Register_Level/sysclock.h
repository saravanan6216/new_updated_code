

#include "stm32l4xx.h"                  // Device header


void sysclock80MHZ()
{
	
	/* Enabling the HSI clock - If not enabled and ready */
  if( (RCC->CR & RCC_CR_HSIRDY) == 0) 
  {
    RCC->CR |= RCC_CR_HSION;  /* HSION=1 */

    /* Waiting until HSI clock is ready */
    while( (RCC->CR & RCC_CR_HSIRDY) == 0);
  }
	RCC->PLLCFGR=0;
	 RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI |  // PLL source is HSI
                   //RCC_PLLCFGR_PLLM_0|RCC_PLLCFGR_PLLM_1 |      // Division factor for PLLM1 
                   RCC_PLLCFGR_PLLN_3 |RCC_PLLCFGR_PLLN_1 ;    // Multiplication factor for PLLN10
                  // RCC_PLLCFGR_PLLR_1       // Division factor for PLLR2

    // Enable PLL and wait for it to be ready
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Flash latency configuration
    FLASH->ACR = FLASH_ACR_LATENCY_4WS; // Adjust according to your setup

    // Select PLL as system clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    // Configure other clock dividers as needed
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;

    // Update SystemCoreClock variable
    //SystemCoreClock = 80000000; // Adjust the value accordingly
	
}
void sys()
{
	
		/* Enabling the HSI clock - If not enabled and ready */
  if( (RCC->CR & RCC_CR_HSIRDY) == 0) 
  {
    RCC->CR |= RCC_CR_HSION;  /* HSION=1 */

    /* Waiting until HSI clock is ready */
    while( (RCC->CR & RCC_CR_HSIRDY) == 0);
  }

  /* Select AHB prescaler to 1 */
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

  /* APB1 prescaler to 1 */
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;

  /* APB2 prescaler to 1 */
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

  /* Select the HSI as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= RCC_CFGR_SW_HSI;

  /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
  FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

  /* Disabling HSE Clock*/
  RCC->CR &= ~RCC_CR_HSEON;
}