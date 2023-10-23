#ifndef __LCD_H
#define __LCD_H

#include "stm32f1xx_hal.h"
void lcd_init (void);
void lcd_send_data (char data);
void lcd_send_cmd (char cmd);
void lcd_send_string (char *str);
void lcd_print_integer_number(int number);
void lcd_print_float_number(float number);
void lcd_gotoxy(uint8_t row, uint8_t column);
void lcd_reset();

#define SLAVE_ADDRESS_LCD 0x4E

#endif