#include "stm32l4xx.h"
#include "usart_h.h"

void sys(void);
void ADC_Init(void);
void GPIO_Init(void);
void ADC_dis();
void ADC_start_conv(void);
uint32_t readADC_value(void);
uint32_t adcValue;
void DMA_Init (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable DMA clock
	2. Set the DATA Direction
	3. Enable/Disable the Circular Mode
	4. Enable/Disable the Memory Increment and Peripheral Increment
	5. Set the Data Size
	6. Select the channel for the Stream
	************************************************/
	
	// Enable the DMA2 Clock
	RCC->AHB1ENR |=RCC_AHB1ENR_DMA1EN;  // DMA2EN = 1
	
	// Select the Data Direction
	DMA1_Channel1->CCR &= ~(1U<<4);  // Peripheral to memory
	
	// Select Circular mode
	DMA1_Channel1->CCR |= (1U<<5);  // CIRC = 1
	
	// Enable Memory Address Increment
	DMA1_Channel1->CCR |= (1U<<7);  // MINC = 1;

	// Set the size for data 
	DMA1_Channel1->CCR |= (1U<<10)|(1<<8); // PSIZE = 01, MSIZE = 01, 16 bit data
	
	// Select channel for the stream
	DMA1_CSELR->CSELR &= ~(0xF<<0);  // Channel 0 selected
	
}
void DMA_Config (uint32_t srcAdd, uint32_t destAdd, uint16_t size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Set the Data Size in the NDTR Register
	2. Set the Peripheral Address and the Memory Address
	3. Enable the DMA Stream
		 
		 Some peripherals don't need a start condition, like UART, So as soon as you enable the DMA, the transfer will begin
		 While Peripherals like ADC needs the Start condition, so Start the ADC later in the program, to enable the transfer
	************************************************/
	
	DMA1_Channel1->CNDTR = size;   // Set the size of the transfer
	
	DMA1_Channel1->CPAR = srcAdd;  // Source address is peripheral address
	
	DMA1_Channel1->CMAR = destAdd;  // Destination Address is memory address
	
	// Enable the DMA Stream
	DMA1_Channel1->CCR |= (1<<0);  // EN =1
}
/* should be used, if you do not wat the circular Mode in DMA 
	 DISABLE the Circular mode to use this.
	 It can be called at any point in the program, and once the conversion is complete, the DMA will stop.
*/
void DMA_Go (uint16_t datasize)
{
	/* If the OverRun occurs (OVR=1)
			Disable and Re-Enable the ADC to prevent Data Curruption
	*/
	
	if ((ADC1->ISR) &(1<<5))
	{
		ADC1->CR &= ~(1<<0);   // ADON =0 Disable ADC1
		
		ADC1->CR |= 1<<0;   // ADON =1 enable ADC1
		
	}
	
	/* To start the DMA again, we need to update the NDTR Counter
		  and also the Interrupt Flags must be cleared
	
			NDTR can only be Updated while the DMA is Disabled
	*/
	
	// Disable the DMA2
	DMA1_Channel1->CCR &= ~(1<<0);
	
	// Clear the Interrupt pending flags. This is important before restarting the DMA
	DMA1->IFCR = 0xfffffff;
	//DMA1->HIFCR = 0xffffffff;
	
	// Set the data size in NDTR Register
	DMA1_Channel1->CNDTR = datasize;
	
	// Enable the DMA2
	DMA1_Channel1->CCR |= 1<<0;
	
	// Start the ADC again
	 ADC_start_conv();
	
}
float volt_convert(int d)
{
	float volt;
	volt=0.0008056 * d;
	return volt;
}
uint16_t RxData[3];
int main(void)
{
	 volatile unsigned int adcValue=0;
	int a=64;
    // Initialize system
    SetSystemClockTo16Mhz();
		DMA_Init ();
	ADC_Init();
	
    // Initialize ADC and GPIO
    
    GPIO_Init();
	ConfigureTimer3();
	GPIO_UART_init();
    while (1)
    {
        //ADC_start_conv();
				DMA_Config ((uint32_t ) &ADC1->DR, (uint32_t) RxData, 3);
			DMA_Go (3);
        // Read ADC value
     adcValue = readADC_value();
			 ADC_dis();
			delay(1000);
			//integer(RxData[0]);
			floatt(volt_convert(RxData[0]));
			tx('V');
			tx(' ');
			
    }
}

uint32_t readADC_value()
{
	int data;
	
	while (!(ADC1->ISR & ADC_ISR_EOC)){};

        // Read ADC value
	data=ADC123_COMMON->CDR;
        return data;
	
}
void ADC_start_conv()
{
	
	// Start ADC conversion
        ADC1->CR |= 0x10000005;
}
void ADC_dis()
{
	ADC1->CR &= ~0x00000001;
	
}
void ADC_Init(void)
{
    // Enable ADC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Configure ADC clock (PCLK2 divided by 2)
    ADC123_COMMON->CCR =0x00020000;//|= ADC_CCR_CKMODE_0;

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
		ADC1->CFGR =0x80000003;
    // Configure ADC channel
    //ADC1->CHSELR = ADC_CHSELR_CHSEL0; // Channel 0
		ADC1->SQR1|=(1<<6);
    // Enable ADC
    //ADC1->CR |= ADC_CR_ADEN;
    //while (!(ADC1->ISR & ADC_ISR_ADRDY));
}

void GPIO_Init(void)
{
    // Enable GPIOC clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // Configure PC0 as analog input
    GPIOC->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1;
	GPIOC->ASCR|=1;
}

