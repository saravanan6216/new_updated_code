#include <LPC214x.h>
int count=0;
void INT0ISR(void)__irq;

void init_interrupt()
{
	VICIntEnable = 0x01<<14;
	VICVectCntl0 =(0x01<<5)|14;
	VICVectAddr0|=(unsigned)INT0ISR;
	
	PINSEL1|=0x01;
	EXTMODE|=0x01;
	EXTPOLAR|=0x00;
	
}

void INT0ISR(void)__irq
{
	long int temp;
	temp=EXTINT;
	if((temp&(0x01))==1)
	{
	    if(count>10)
	    {		
	      {
	        IO1SET|=(0x01)<<25;
	        }
	       if(count==15)
			    {
				    count=0;IO1CLR|=(0x01)<<25;
			    }
		  }
		else
	      {
		    count++;
		    IO1CLR|=(0x01)<<25;
		   }
	}
	EXTINT=temp;
	VICVectAddr=0x00;
}

int main()
{
   init_interrupt();
   
   while(1)
   {
   }

}
