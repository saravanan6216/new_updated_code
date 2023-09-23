#include<LPC214x.h>

void init_PLL()
{
	PLL0CON=0x01;//Enable clock
	PLL0CFG=0x24;//set up pll for 60Mhz
	PLL0FEED=0xAA;
	PLL0FEED=0x55;
	while((PLL0STAT & 0x400)==0);
	
	PLL0CON=0x03;
	PLL0FEED=0xAA;
	PLL0FEED=0x55;
	VPBDIV=0x01;
}

void delay_ms(unsigned int m)
{
	T0CTCR=0x00;
	T0PR=60000-1;
	T0TC=0;
	T0TCR=0x01;//ENABLE TIMER
	while(T0TC < m);
	
	T0TCR=0x00;//disable timer
}

void delay_us(unsigned int m)
{
	T0CTCR=0x00;
	T0PR=60-1;
	T0TC=0;
	T0TCR=0x01;//ENABLE TIMER
	while(T0TC < m);
	
	T0TCR=0x00;//disable timer
}


void UART_init()
{
	PINSEL0=0x05;
	U0LCR=0x83;
	U0DLL=0x87;
	U0DLM=0x01;
	U0LCR=0x03;
}


void UART_TX(unsigned char my_data)
{
	U0THR=my_data;
	while((U0LSR & (0x01<<5))==0);
}


void UART_TX_string(unsigned char *my_data)
{
	while(*my_data!=0)
	{
		UART_TX(*my_data);
		my_data++;
	}
}


unsigned char UART_RX(void)
{
	unsigned char my_data;
	while((U0LSR & (0x01<<0))==0);
	
	my_data=U0RBR;
	return my_data;
}


int data;
	
int main()
{
	
	IO0DIR|=(0x01)<<4;
	IO1DIR&=~(0x01)<<1;
	init_PLL();
	UART_init();
	
	
	
	while(1)
	{
		UART_TX('e');
		delay_ms(1000);
		data=UART_RX();
		delay_ms(1000);
	}
	
}

