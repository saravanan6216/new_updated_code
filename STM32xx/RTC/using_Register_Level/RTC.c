
#include "stm32l4xx.h"                  // Device header                

typedef struct{
  uint8_t Hours;           
  uint8_t Minutes;         
  uint8_t Seconds;         
	
}RTC_TimeTypeDef;


typedef struct{
  uint8_t Day;    
  uint8_t Month;     
  uint8_t Year;  
	
}RTC_DateTypeDef;


	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

void RTC_Init(void){
	
	 RCC->APB1ENR1|=(1<<10);

	//RCC->APB1ENR |= RCC_APB1ENR_PWREN;					//enable peripheral clock power
	PWR->CR1 |= (1<<8);										//enable access to the RTC registers
	
	RCC->CSR |= RCC_CSR_LSION;									//enable LSI
	while(!(RCC->CSR & RCC_CSR_LSIRDY));				//wait for LSI ready flag
	RCC->BDCR &= ~((2U<<8)|(1U<<15));
	RCC->BDCR |= (2<<8)|(1<<15);												//select LSI and enable RTC
	
	RTC->WPR |= 0xCA;														//enter key to unlock write protection
	RTC->WPR |=	0x53;
	
	 RTC->ISR |=(1<<7);
	while(!(RTC->ISR & (1<<6)));					//wait for RTC init ready flag
	
	RTC->PRER |=0xFF;
	RTC->PRER |=0x7F<<16;
	
	RTC->TR |= 0x00113500;						//set hour as 18
	//RTC->TR |= 0x0000;							//set minute as 0
	RTC->DR |= 0x00234919;							//SET date as 19/09/23 thuesday
	RTC->CR |= RTC_CR_BYPSHAD;
	
	RTC->ISR &= ~RTC_ISR_INIT;			//clear init bit
	
	PWR->CR1 &= ~(1U<<8);					// disable access to RTC registers

}


void RTC_get_time (void){
	sTime.Hours=((RTC->TR >> 20)*10) + ((RTC->TR >>16) & 0xf);
	sTime.Minutes=((RTC->TR >> 12) & 0x3)*10 + ((RTC->TR >>8) & 0xf);
	sTime.Seconds=((RTC->TR >> 4) & 0x3)*10 + (RTC->TR & 0xf);
	

}

void RTC_get_date(void){

	sDate.Year=((RTC->DR >> 20)*10) + ((RTC->DR >>16) & 0xf);
	sDate.Month=((RTC->DR >> 12) & 0x1)*10 + ((RTC->DR >>8) & 0xf);
	sDate.Day=((RTC->DR >> 4) & 0x3)*10 + (RTC->DR & 0xf);

}

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

void sysclock80MHZ()
{
	
	
   RCC->APB1ENR1 |= (1<<28);				// POWER ENABLE 
	 PWR->CR1 |= (1<<9); 						//POWER RANGE 1
	 RCC->PLLCFGR=0; 								//RESET
	 RCC->PLLCFGR  = 2 ;  					// PLL source is HSI
	 RCC->PLLCFGR |= (10<<8)|(1<<24);      // PLLN MULTIPLIER 10 AND PLL OUTPUT ENABLE 
	 RCC->PLLCFGR &= ~(1U<<25) | ~(1U<<26) |  ~(1U<<4) | ~(1U<<4) |  ~(1U<<6); //PLLR DIVIDED BY 2  & PLLM DIVIDED BY 1
	
    // Enable PLL and wait for it to be ready
	 if( (RCC->CR & RCC_CR_PLLRDY) == 0) {
		 
    RCC->CR |= (1<<24);   // PLLON
		 
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
	}	
	
    // Flash latency configuration
    FLASH->ACR = FLASH_ACR_LATENCY_4WS; // Adjust according to your setup

    // Configure other clock dividers as needed
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 | RCC_CFGR_PPRE2_DIV1;

    
		 RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW_PLL));
	
		// Select PLL as system clock source
		 RCC->CFGR |= 3;
	 while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
		  
		 
	
}

static void SetSystemClockTo16Mhz(void)
{
  // Enabling the HSI clock - If not enabled and ready 
  if( (RCC->CR & RCC_CR_HSIRDY) == 0) 
  {
    RCC->CR |= RCC_CR_HSION;  // HSION=1 
  
    // Waiting until HSI clock is ready 
    while( (RCC->CR & RCC_CR_HSIRDY) == 0);
  }
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
  
  // APB1 prescaler to 1 
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
  
  // APB2 prescaler to 1 
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;
 
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW)); // RESET HSI FROM SYSTEM SOURCE

  
  // Disabling HSE Clock
  RCC->CR &= ~RCC_CR_HSEON;
}


static void ConfigureTimer3(void)
{
  /* Enable the APB clock FOR TIM3  */
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN  ;
  
  /* fCK_PSC / (PSC[15:0] + 1)
     (16 MHz / (15+1)) = 1 MHz timer clock speed */
  TIM3->PSC = 79;
   // Select AHB prescaler to 1 
  
  /* (1 MHz / 1000) = 1KHz = 1ms */
  /* So, this will generate the 1ms delay */
  TIM3->ARR = 999;
  
  /* Finally enable TIM3 module */
  TIM3->CR1 = (1 << 0);
}


int main(void)
{

  /* Set System clock to 16 MHz using HSI */
   SetSystemClockTo16Mhz();
   sysclock80MHZ();
  /* Configure the Timer 3 */
  ConfigureTimer3();
  RTC_Init();
  
  /* Endless loop */
  while(1)
  {
    RTC_get_date();
		RTC_get_time();

    delay(1000);
  }
}

