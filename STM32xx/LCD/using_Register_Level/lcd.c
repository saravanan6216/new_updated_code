#include "stm32l4xx.h"                  // Device header

#define RS 8
#define EN 9


#define PORTA GPIOA->ODR
#define PORTB GPIOB->ODR
#define PORTC GPIOC->ODR


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
  RCC->CR &= ~RCC_CR_HSEON;
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


enum {
	PA,
	PB,
	PC
};

void GPIO_INIT(int port)
{
	if(port==PA)
	{
		RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN;
		GPIOA->MODER&=0;
		GPIOA->MODER|=0x55555;
	}
	else if(port==PB)
	{
		RCC->AHB2ENR|=RCC_AHB2ENR_GPIOBEN;
		GPIOB->MODER&=0;
		GPIOB->MODER|=0x55555;
		GPIOB->OSPEEDR&=0;
		GPIOB->PUPDR&=0;
		
	}
	
	else if(port==PC)
	{
		RCC->AHB2ENR|=RCC_AHB2ENR_GPIOCEN;
		GPIOC->MODER&=0;
		GPIOC->MODER|=0x55555;
	}
}

void LCD_cmd(unsigned char cmd)
{
	PORTB &=0xFFFFFC0F;
	PORTB |=cmd&0xF0;
	PORTB &= ~(1U<<RS);
	PORTB |=(1U<<EN);
	delay(2);
	PORTB &= ~(1U<<EN);
	
	
	PORTB &=0xFFFFFC0F;
	PORTB |=(cmd<<4)&0xF0;
	PORTB &= ~(1U<<RS);
	PORTB |=(1U<<EN);
	delay(2);
	PORTB &= ~(1U<<EN);
	
}
void LCD_data(unsigned char data)
{
	PORTB &=0xFFFFFC0F;
	PORTB |=data&0xF0;
	PORTB |= (1U<<RS);
	PORTB |=(1U<<EN);
	delay(2);
	PORTB &= ~(1U<<EN);	
	
	
	PORTB &=0xFFFFFC0F;
	PORTB |=(data<<4)&0xF0;
	PORTB |= (1U<<RS);
	PORTB |=(1U<<EN);
	delay(2);
	PORTB &= ~(1U<<EN);	
}
void LCD_string(unsigned char *p)
{
    while(*p!='\0') {
       LCD_data(*p++);
    }
}
void LCD_init()
{
	delay(20);
		LCD_cmd(0x28);
    LCD_cmd(0x0E);
    LCD_cmd(0x06);
    LCD_cmd(0x01);
    LCD_cmd(0x80);
}

int main()
{
	SetSystemClockTo16Mhz();
	ConfigureTimer3();
	unsigned char *p="helloooo";
	unsigned char *pt="world";
	GPIO_INIT(PB);
	LCD_init();
	LCD_cmd(0x80);
	LCD_string(p);
	LCD_cmd(0xC2);
	LCD_string(pt);
	while(1)
		//LCD_cmd(0x18);
		delay(2);
	
}
