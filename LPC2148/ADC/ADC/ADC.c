#include<LPC214x.h>
#include <LCD.h>
#include <stdio.h>
#include <string.h>


void ADC0_Init(void);
unsigned int ADC0_Read(void);
unsigned long adc_data;
int main()
{
	LCD_Init(); 
	delay_ms(10);
	ADC0_Init();
	while(1)
	{
		adc_data = ADC0_Read();
		adc_data = adc_data*3300;
		adc_data = adc_data/1023;			//Value of Voltage in Milli Volts
		Display_Number_Lcd(2,1,4,adc_data);
		LCD_Char('m');
		LCD_Char('V');
		delay_ms(10);
	}	
}

unsigned long y;
char b[45];
void ADC0_Init(void)
{

	AD0CR = 1<<21;								//A/D is Operational
	AD0CR = 0<<21;								//A/D is in Power Down Mode
	PCONP = (PCONP &0x001817BE) | (1UL<<12);
	PINSEL0 = 0x00;
	PINSEL1 = 0x00400000;					//P0.27 is Configured as Analog to Digital Converter Pin AD0.0
	
	AD0CR = 0x00200401;						//CLKDIV=4,Channel-0.0 Selected,A/D is Operational

}

unsigned int ADC0_Read(void)
{
	unsigned long adc_data;
	
	AD0CR |= 1UL<<24;							//Start Conversion
	do
	{
		adc_data = AD0GDR;
	}while(!(adc_data & 0x80000000));

	AD0CR &= ~0x01000000;					//Stop Conversion   
	
	adc_data = adc_data >> 6;
	adc_data = adc_data & 0x3FF;    //Clear all Bits

return (adc_data);
}


	


