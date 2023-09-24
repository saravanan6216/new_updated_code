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


/**** STEPS FOLLOWED  ************
1. Enable the I2C CLOCK and GPIO CLOCK
2. Configure the I2C PINs for ALternate Functions
	a) Select Alternate Function in MODER Register
	b) Select Open Drain Output 
	c) Select High SPEED for the PINs
	d) Select Pull-up for both the Pins
	e) Configure the Alternate Function in AFR Register
3. Reset the I2C 	
4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
5. Configure the clock control registers
6. Configure the rise time register
7. Program the I2C_CR1 register to enable the peripheral
*/
static void GPIOCONFIG()
{
	RCC->APB1ENR1 |=(1<<21);						//Enable the I2C CLOCK
	RCC->AHB2ENR |=(1<<1);							//Enable the GPIO CLOCK for i2c pins
	GPIOB->MODER =0;
	GPIOB->MODER |=(2<<16)|(2<<18);			//Select Alternate Function in MODER Register for i2c pins pb8 and pb9
	
	GPIOB->OTYPER |= (1<<8)|(1<<9);			//Select Open Drain Output for pb8 and 9
	GPIOB->OSPEEDR=0;
	GPIOB->OSPEEDR |=(3<<16)|(3<<18);		// Select High SPEED for the PINs
	
	GPIOB->PUPDR |= (1<<16)|(1<<18);		//Select Pull-up for both the Pins
	
	GPIOB->AFR[1] |= (4<<0)|(4<<4);			// Configure the Alternate Function in AFR Register 4 for i2c
}
static void i2c_init()
{
	I2C1->CR1=0;//reset
 // I2C1->CR1	|= ((1U<<8));
	I2C1->TIMINGR=0;
	I2C1->TIMINGR |= (3<<28)|(0x4<<20)|(0x2<<16)|(0xF<<8)|(0x13<<0); //seting the mode 
	//I2C1->CR1 &= ~(1U<<17);//clock stretching
	//I2C1->CR1|=(1<<12);//disable filter
	//I2C1->CR1|=(1<<17);
	
	I2C1->CR1 |= (1<<0);
	

}

/*void master_init()
{
	//I2C1->CR1 |=(1<<6);
	
	//I2C1->CR2 &=~(1U<<11);
	I2C1->CR2 |=(0x8<<1);//|(1<<13);
	//I2C1->CR2 &= ~(1U<<10);
	//I2C1->CR2 |=(0xFF<<16);
	
	I2C1->CR2 |=(1<<13);
	
}
void ms(unsigned int a)
{
	while((I2C1->ISR & (1<<0))==0);
	I2C1->TXDR=a;
	while((I2C1->ISR & (1<<6))==0);
	
} 
int main()
{
	SetSystemClockTo16Mhz();
	ConfigureTimer3();
	 GPIOCONFIG();

	i2c_init();
	*/
	/* *** STEPS FOLLOWED  ************
1. Enable the ACK
2. Send the START condition 
3. Wait for the SB ( Bit 0 in SR1) to set. This indicates that the start condition is generated
*
	master_init();
	while(1)
	{
		ms(5);
		delay(1000);
	}
}*/


void I2C_Start(unsigned int add);
void Wait_For_Start(void);
void I2C_Send_Addr(unsigned char address);
void Check_Addr_Bit(void);
void I2C_write(unsigned char a);
void DR_Check(void);
void I2C_Send_M_Addr(char address);
void Dis_Ack(void);
void I2C_Stop(void);
void Wait_RxNE(void);
char Receive_data(void);
char I2C_ReadByte(char saddr, char maddr);
int main()
{
	SetSystemClockTo16Mhz();
	ConfigureTimer3();
	 GPIOCONFIG();
  i2c_init();
	
			
  //Check_Addr_Bit();

	while(1)
	{
		//I2C_Send_Addr(0x8);
		 I2C_write(0x5);
		delay(1000);
//I2C_Stop();
	

	}
}

void I2C_Start(unsigned int add)
{
	I2C1->CR2|=(1<<16)|(1<<25)|(1U<<13)|(add<<1);
	//I2C1->CR2|=(1<<25);
	//I2C1->CR2&=~(1U<<10);
	///I2C1->CR2 |= (add<<13);
}
void Wait_For_Start(void)
{
	while(!(I2C1->ISR &(1<<15)));
}
void I2C_Send_Addr(unsigned char address)
{
	
	I2C1->CR2 |=(address<<1);
}
void Check_Addr_Bit(void)
{
	while(!(I2C1->ISR&(1<<3)));
	//I2C1->ICR|=(1<<0);
}

void I2C_write(unsigned char a)
{
	I2C_Start(8);
  Wait_For_Start();
	while(!(I2C1->ISR&(1<<0)));
	I2C1->TXDR=a;
	while(!(I2C1->ISR&(1<<6)));
	//while(!(I2C1->ISR&(1<<0)));
	I2C_Stop();
}

void I2C_Stop(void)
{
	I2C1->CR2|=(1<<14);
	
}
void Wait_RxNE(void);
char Receive_data(void);
char I2C_ReadByte(char saddr, char maddr);	
	
	
	
	
	
	
	
	
