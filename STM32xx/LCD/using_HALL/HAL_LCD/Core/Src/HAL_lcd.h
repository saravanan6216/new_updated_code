/*
 * HAL_lcd.h
 *
 *  Created on: Jul 28, 2023
 *      Author: DELL
 */

#ifndef SRC_HAL_LCD_H_
#define SRC_HAL_LCD_H_
int init_lcd_HAL();
void port(int data);
void HAL_data(char d);
void cmd(uint8_t c);
void string(char *s);

//#define RS 0;
//#define EN 1;
#define RS_EN HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, SET)
#define RS_DIS HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0, RESET)
#define EN_EN HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, SET)
#define EN_DIS HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1, RESET)
int init_lcd_HAL()
{
		HAL_Delay(50);	/* LCD Power ON Initialization time >15ms */
		cmd(0x02);	/* 4bit mode */
		cmd(0x28);	/* Initialization of 16X2 LCD in 4bit mode */
		cmd(0x0E);	/* Display ON Cursor OFF */
		cmd(0x06);	/* Auto Increment cursor */
		cmd(0x01);	/* clear display */
		cmd(0x80);	/* cursor at home position */

}
void port(int data)
{
	if(data&(1<<4))
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4, SET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4, RESET);
	if(data&(1<<5))
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, SET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5, RESET);
	if(data&(1<<6))
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, SET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6, RESET);
	if(data&(1<<7))
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, SET);
	else
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7, RESET);
}
void HAL_data(char d)
{
		port(d&0xf0);
		RS_EN;
		EN_EN;
		HAL_Delay(100);
		EN_DIS;
		HAL_Delay(100);

		port(d&0x0f);
		RS_DIS;
		EN_EN;
		HAL_Delay(100);
		EN_DIS;
		HAL_Delay(100);

}
void cmd(uint8_t c)
{
	port(c&0xf0);
	RS_DIS;
	EN_EN;
	HAL_Delay(100);
	EN_DIS;
	HAL_Delay(100);

	port(c&0x0f);
	RS_DIS;
	EN_EN;
	HAL_Delay(100);
	EN_DIS;
	HAL_Delay(100);




}
void string(char *s)
{
	while(*s!='\0')
	{
		data(*s);
		s++;
	}

}
#endif /* SRC_HAL_LCD_H_ */
