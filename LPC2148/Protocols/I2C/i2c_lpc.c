#include <lpc214x.h>
#include <stdio.h>

#define I2C_SCL_PIN  0 
#define I2C_SDA_PIN  1 

#define I2C_WRITE    0
#define I2C_READ     1

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

void i2c_init()
{
	PINSEL0|=0x50;
    I2C0CONSET = (1 << 6);  // Enable I2C module
    I2C0SCLH = 75;           
    I2C0SCLL = 75;         
}

void i2c_start(void)
{
    I2C0CONSET = (1 << 5);  // Start 
    while (!(I2C0CONSET & (1 << 3)));  // Wait 
    I2C0CONCLR = (1 << 3);  // Clear 
}

void i2c_stop(void)
{
    I2C0CONSET = (1 << 4);  // Stop 
    I2C0CONCLR = (1 << 3);  // Clear  bit
}

void i2c_write_byte(int data)
{
    I2C0DAT = data;  // Write data to transmit
    I2C0CONCLR = (1 << 3);  // Clear 
    while (!(I2C0CONSET & (1 << 3)));  // Wait 
}

int i2c_read_byte(void)
{
    I2C0CONSET = (1 << 2);  // Enable ACK
    I2C0CONCLR = (1 << 3);  // Clear 
    while (!(I2C0CONSET & (1 << 3)));  
    return I2C0DAT;  
}

void i2c_write(int slave_address, int *data, int length)
{
    int i;
    i2c_start();
    i2c_write_byte((slave_address << 1) | I2C_WRITE);

    for (i = 0; i < length; i++)
    {
        i2c_write_byte(data[i]);
    }

    i2c_stop();
}

void i2c_read(int slave_address, int *data, int length)
{
    int i;

    i2c_start();
    i2c_write_byte((slave_address << 1) | I2C_READ);

    for (i = 0; i < length - 1; i++)
    {
        data[i] = i2c_read_byte();
    }
    
    I2C0CONCLR = (1 << 2);  // Disable ACK for the last byte
    data[length - 1] = i2c_read_byte();

    i2c_stop();
}

int i;
int main(void)
{
    int slave_add = 0x50;
    int dataw[4] = {0x00, 0x01, 0x02, 0x03};
    int datar[4];
	init_PLL();
    i2c_init();
    
  while(1)
  {  
    i2c_write(slave_add, dataw, sizeof(dataw));
    i2c_read(slave_add, datar, sizeof(datar));
    for (i = 0; i < sizeof(datar); i++)
    {
       printf("Read Data[%d]: 0x%02X\n", i, datar[i]);
    }
   }
    return 0;
}

