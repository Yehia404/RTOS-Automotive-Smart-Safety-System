#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"
#include "i2c.h"
#include "delay.h"

// I2C LCD Module Address (typically 0x27 or 0x3F for most I2C LCD backpacks)
#define LCD_ADDR 0x27
#define LCD_WRITE_ADDR (LCD_ADDR << 1)

// LCD commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// LCD flags
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00
#define LCD_8BITMODE 0x10
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// Bit positions for LCD control bits in I2C byte
#define En 0x04 // Enable bit
#define Rw 0x02 // Read/Write bit
#define Rs 0x01 // Register select bit

// Global LCD power state flag
extern bool lcd_powered;

// Function prototypes
void LCD_Write_Nibble(uint8_t nibble, uint8_t rs);
void LCD_Write_Byte(uint8_t data, uint8_t rs);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_Power_On(void);
void LCD_Power_Off(void);
bool LCD_Is_Powered(void);

#endif // __LCD_H__