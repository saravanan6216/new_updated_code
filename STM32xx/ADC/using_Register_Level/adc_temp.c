#include "stm32l4xx.h"
#include "lcd.h"
#include <stdlib.h>
void sys(void);
void ADC_Init(void);
void GPIO_Init(void);
void ADC_dis();
void ADC_start_conv(void);
uint32_t readADC_value(void);
uint32_t adcValue;

int main(void)
{
	 volatile unsigned int adcValue;
	int a=64;
    // Initialize system
	
    sys();
    // Initialize ADC and GPIO
    ADC_Init();
    //GPIO_Init();
	ConfigureTimer3();
	unsigned char *p;
	//GPIO_INIT(PB);
	//LCD_init();
	//LCD_cmd(0x89);
	
	GPIO_INIT(PB);
  LCD_init();
    while (1)
    {
        ADC_start_conv();

       // int adc;
	//adc=(int)ADC1->DR;

        // Read ADC value
        adcValue = readADC_value();
			integer(adcValue);
			LCD_cmd(0x80);
		/*	p=(unsigned char*)adcValue;
			LCD_string(p);
			LCD_cmd(0x01);
			LCD_cmd(0x80);
			delay(1000);*/
			 ADC_dis();
			delay(1000);
				//LCD_cmd(0xC0);
			//LCD_string(adcValue);
        // Process the ADC value here
    }
}
uint32_t readADC_value()
{
	
		while (!(ADC1->ISR & ADC_ISR_EOC));

        // Read ADC value
	
//uint32_t adc2=(uint32_t)ADC1->DR*10000;
        return ADC1->DR;
	
}
void ADC_start_conv()
{
	
	// Start ADC conversion
        ADC1->CR |= 0x10000005;
}
void ADC_dis()
{
	ADC1->CR &= ~0x00000001U;
	
}
void ADC_Init(void)
{
    // Enable ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Configure ADC clock (PCLK2 divided by 2)
    ADC123_COMMON->CCR =0x00810000;//|= ADC_CCR_CKMODE_0;

    // Enable ADC voltage regulator
    //ADC1->CR |= ADC_CR_ADVREGEN;

    // Wait for ADC voltage regulator startup
    //while (!(ADC1->ISR & ADC_ISR_REGENRDY));

    // Calibrate ADC
    ADC1->CR =0;
    //while (ADC1->CR & ADC_CR_ADCAL);

    // Configure ADC settings
   // ADC1->CFGR &= ~ADC_CFGR_CONT; // Single conversion mode
   // ADC1->CFGR &= ~ADC_CFGR_ALIGN; // Right alignment
 ADC1->CFGR =0x80000000;//0x80000000;
    // Configure ADC channel
    //ADC1->CHSELR = ADC_CHSELR_CHSEL0; // Channel 0
		ADC1->SQR1|=(17<<6);
    // Enable ADC
    //ADC1->CR |= ADC_CR_ADEN;
    //while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

void GPIO_Init(void)
{
    // Enable GPIOA clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Configure PA0 as analog input
    GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1;
	GPIOC->ASCR|=1;
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