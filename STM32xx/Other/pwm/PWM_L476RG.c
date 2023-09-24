#include "stm32l4xx.h"                  // Device header

 

static void delay( uint32_t ms )
{
  uint32_t i;
  for( i = 0; i <= ms; i++ )
  {
    /* Clear the count */
    TIM3->CNT = 0;

    /* Wait UIF to be set */
    while((TIM3->SR & TIM_SR_UIF) == 0);    /* This will generate 1ms delay */

    /* Reset UIF */
    TIM3->SR &= ~TIM_SR_UIF;
  }
}

 

static void SetSystemClockTo16Mhz(void)
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
 // RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= RCC_CFGR_SW_HSI;

  /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
  //FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

  /* Disabling HSE Clock*/
  //RCC->CR &= ~RCC_CR_HSEON;
}

 

 

static void ConfigureTimer3(void)
{
  /* Enable the APB clock FOR TIM3  */
  SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TIM3EN);

  /* fCK_PSC / (PSC[15:0] + 1)
     (16 MHz / (15+1)) = 1 MHz timer clock speed */
  TIM3->PSC = 15;

  /* (1 MHz / 1000) = 1KHz = 1ms */
  /* So, this will generate the 1ms delay */
  TIM3->ARR = 1000-1;

  /* Finally enable TIM3 module */
  TIM3->CR1 = (1 << 0);
}

 static void ConfigureTimer2(void)
{
  /* Enable the APB clock FOR TIM3  */
  SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_TIM2EN);

	TIM2->CCMR1 |=(6<<4)|TIM_CCMR1_OC1PE;
	
	TIM2->CR1 |=TIM_CR1_ARPE;
	TIM2->CCER  &= (~TIM_CCER_CC1P);
	
	TIM2->CCER |=  TIM_CCER_CC1E;
  /* fCK_PSC / (PSC[15:0] + 1)
     (16 MHz / (15+1)) = 1 MHz timer clock speed */
  TIM2->PSC = 15;
  /* (1 MHz / 1000) = 1KHz = 1ms */
  /* So, this will generate the 1ms delay */
  TIM2->ARR = 1000-1;
	

  /* Finally enable TIM3 module */
  TIM2->CR1 = (1 << 0);
}

 

int main(void)
{

 

  /* Set System clock to 16 MHz using HSI */
  SetSystemClockTo16Mhz();

  /* Configure the Timer 3 */
  ConfigureTimer3();
	ConfigureTimer2();
  /* Enable the AHB clock all GPIO port A */
  SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);

 

	
  /* set all Port C as output */
  GPIOA->MODER = (1<<11);
	GPIOA->AFR[0]|=(1<<20);
 
uint16_t b=0;
  /* Endless loop */
	while(1){
  while(b<999)
  {
   
		b+=10;
		TIM2->CCR1 = b;
    delay(10);
  }
	 while(b>0)
  {
   
		b-=10;
		TIM2->CCR1 = b;
    delay(10);
  }
}
}

