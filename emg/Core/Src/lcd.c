/*
 * lcd.c
 *
 */

#include "lcd.h"
#include "stm32f0xx_hal.h"

#define LCD_E_PIN	GPIO_PIN_7
#define LCD_E_PORT	GPIOB

#define LCD_RS_PIN	GPIO_PIN_9
#define LCD_RS_PORT	GPIOA

#define LCD_D4_PIN	GPIO_PIN_5
#define LCD_D4_PORT	GPIOB

#define LCD_D5_PIN	GPIO_PIN_4
#define LCD_D5_PORT	GPIOB

#define LCD_D6_PIN	GPIO_PIN_10
#define LCD_D6_PORT	GPIOB

#define LCD_D7_PIN	GPIO_PIN_8
#define LCD_D7_PORT	GPIOA

#define LCD_LIGHT_PIN GPIO_PIN_6
#define LCD_LIGHT_PORT GPIOB


#define LCD_E_HIGH HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET)
#define LCD_E_LOW HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET)
#define LCD_RS_LOW HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET)
#define LCD_RS_HIGH HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET)



#define LCD_LINE1 		0x00
#define LCD_LINE2 		0x40


void lcd_sendHalf(uint8_t data)
{
LCD_E_HIGH;
HAL_GPIO_WritePin(LCD_D4_PORT, LCD_D4_PIN, (data & 0x01));
HAL_GPIO_WritePin(LCD_D5_PORT, LCD_D5_PIN, (data & 0x02));
HAL_GPIO_WritePin(LCD_D6_PORT, LCD_D6_PIN, (data & 0x04));
HAL_GPIO_WritePin(LCD_D7_PORT, LCD_D7_PIN, (data & 0x08));
LCD_E_LOW;
}

void lcd_write_byte(uint8_t data)
{
	lcd_sendHalf(data >> 4);
	lcd_sendHalf(data);
	HAL_Delay(1);
}

void lcd_write_cmd(uint8_t cmd)
{
	LCD_RS_LOW;
	lcd_write_byte(cmd);
}

void lcd_char(char data)
{
	LCD_RS_HIGH;
	lcd_write_byte(data);
}

void LCD_Init(void)
{
    HAL_GPIO_WritePin(LCD_LIGHT_PORT, LCD_LIGHT_PIN,GPIO_PIN_SET);
	HAL_Delay(15);

	LCD_E_LOW;
	LCD_RS_LOW;

	lcd_sendHalf(0x03);
	HAL_Delay(5);

	lcd_sendHalf(0x03);

	HAL_Delay(1);
	lcd_sendHalf(0x03);

	HAL_Delay(1);
	lcd_sendHalf(0x02);

	HAL_Delay(1);

	lcd_write_cmd( LCD_FUNC | LCD_4_BIT | LCDC_TWO_LINE | LCDC_FONT_5x7);
	lcd_write_cmd( LCD_ONOFF | LCD_DISP_ON );
	lcd_write_cmd( LCD_CLEAR );

	HAL_Delay(5);
	lcd_write_cmd( LCDC_ENTRY_MODE | LCD_EM_SHIFT_CURSOR | LCD_EM_RIGHT );

}

void lcd_locate(uint8_t x, uint8_t y)
{

	switch(y)
	{
		case 0:
			lcd_write_cmd( LCDC_SET_DDRAM | (LCD_LINE1 + x) );
			break;

		case 1:
			lcd_write_cmd( LCDC_SET_DDRAM | (LCD_LINE2 + x) );
			break;
	}

}

void lcd_str(char *text)
{
	while(*text)
		lcd_char(*text++);
}


