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
  FLASH->ACR = FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_3WS;

 

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
static void configGPIOSPI()
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(0xffffUL);
    GPIOA->MODER |= (0<<8)|(2<<10)|(2<<12)|(2<<14);//set alternate function for gpio pins
    GPIOA->OSPEEDR |= (2<<10)|(2<<12)|(2<<14);//high speed
    GPIOA->AFR[0]|=(5<<20)|(5<<24)|(5<<28);

}
static void configSPImaster()
{
    RCC->APB2ENR|=RCC_APB2ENR_SPI1EN;
    SPI1->CR1 |=(1<<3);//setting clock
    SPI1->CR1 &= (1<<0)|(1<<1);//CPHA=1,CPOL=1
    SPI1->CR1 &=~(1U<<10);//full duplex
    SPI1->CR1 &=~(1U<<7);//msb first
    SPI1->CR1 |=(1<<13);
    SPI1->CR1 &=~(1U<<11);
    SPI1->CR1 &=~(1U<<2);//set slave

    SPI1->CR2 |=(3<<8);
    SPI1->CR2 |=(1<<2)|(1<<7);
}
static void spi_enable()
{
    SPI1->CR1|=(1<<6);
}
static void ss_enable()
{
    GPIOA->ODR &=~(1U<<4);

}
static void ss_disable()
{
    GPIOA->ODR |=(1<<4);

}
static void tx(unsigned char a)
{
    ss_enable();
    while(!(SPI1->SR & (1<<1)));
    SPI1->DR=a;
    ss_disable();
}
static unsigned char rx()
{
		
    unsigned char a;
    while(!(SPI1->SR & (1<<0)));
    a=(unsigned char)SPI1->DR;
    return a;
		
}
void spi_interrupt_config()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	//EXTI->IMR1|= EXTI_IMR1_IM27;
NVIC_EnableIRQ(SPI1_IRQn);
}
int main()
{
		unsigned char a;
    SetSystemClockTo16Mhz();
    ConfigureTimer3();
    configGPIOSPI();
	GPIOA->MODER |=(1<<4);
	GPIOA->MODER &=~(1U<<5);
    configSPImaster();
    spi_enable();
spi_interrupt_config();
			GPIOA->BSRR|=(1<<2);
    while(1)
    {
			a=rx();
		  delay(1000);
		}
}
void SPI1_IRQHandler()
{
	
	NVIC_DisableIRQ(SPI1_IRQn);
}