#include <lpc214x.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
void LCD_CMD(char);
void LCD_INIT(void);
void LCD_STRING (char*);
void LCD_CHAR (char );

void LCD_CMD(char command)
{
	IO0PIN = ( (IO0PIN & 0xFFFF00FF) | (command<<8) );
	IO0SET = 0x00000040; 
	IO0CLR = 0x00000030; 
	delay_ms(2);
	IO0CLR = 0x00000040; 
	delay_ms(5);
}

void LCD_INIT(void)
{
	IO0DIR = 0x0000FFF0; 
	delay_ms(20);
	LCD_CMD(0x38);  
	LCD_CMD(0x0C);   
	LCD_CMD(0x06);  
	LCD_CMD(0x01);   
	LCD_CMD(0x80); 
}

void LCD_STRING (char* msg)
{
	uint8_t i=0;
	while(msg[i]!=0)
	{
		IO0PIN = ( (IO0PIN & 0xFFFF00FF) | (msg[i]<<8) );
		IO0SET = 0x00000050; 
		IO0CLR = 0x00000020; 
		delay_ms(2);
		IO0CLR = 0x00000040; 
		delay_ms(5);
		i++;
	}
}

void LCD_CHAR (char msg)
{
		IO0PIN = ( (IO0PIN & 0xFFFF00FF) | (msg<<8) );
		IO0SET = 0x00000050; 
		IO0CLR = 0x00000020; 
		delay_ms(2);
		IO0CLR = 0x00000040; 
		delay_ms(5);
}

