#include <lpc214x.h>


void SPI_Init()
{
	PINSEL0 = PINSEL0 | 0x00001500; 
	S0SPCR = 0x0020; 
	S0SPCCR = 0x10; 
}

void SPI_Write(char data)
{
	char flush;
	IO0CLR = (1<<7);  
	S0SPDR = data;  
	while ( (S0SPSR & 0x80) == 0 );  
	flush = S0SPDR;
	IO0SET = (1<<7); 
}

char SPI_Read()
{
	IO0CLR = (1<<7);  
	S0SPDR = 0xFF;  
	while ( (S0SPSR & 0x80) == 0 );  
	IO0SET = (1<<7); 
	return S0SPDR;  
}
int main()
{
  char data="hel0";
  char rec;
 
  SPI_Init();
  
  while(1)
  {
  SPI_Write(data);
  rec=SPI_Read();
  }


}

