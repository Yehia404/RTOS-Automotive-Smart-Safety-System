#include "lcd.h"

// Global LCD power state flag
bool lcd_powered = false;

void LCD_Write_Nibble(uint8_t nibble, uint8_t rs)
{
    // Only perform operations if LCD is powered
    if (!lcd_powered)
    {
        return;
    }

    uint8_t data = (nibble << 4) | LCD_BACKLIGHT;

    if (rs)
    {
        data |= Rs; // Set RS bit for data
    }

    // Send data with enable bit low
    I2C0_SendByte(LCD_WRITE_ADDR, data);
    delay_ms(1);

    // Send with enable bit high
    I2C0_SendByte(LCD_WRITE_ADDR, data | En);
    delay_ms(1);

    // Send with enable bit low again
    I2C0_SendByte(LCD_WRITE_ADDR, data);
    delay_ms(1);
}

void LCD_Write_Byte(uint8_t data, uint8_t rs)
{
    // Only perform operations if LCD is powered
    if (!lcd_powered)
    {
        return;
    }

    // Send high nibble first
    LCD_Write_Nibble(data >> 4, rs);

    // Then send low nibble
    LCD_Write_Nibble(data & 0x0F, rs);
}

void LCD_Init(void)
{
    // Set LCD to powered state
    lcd_powered = true;

    delay_ms(50); // Wait for LCD to power up

    // Special initialization sequence for 4-bit mode
    LCD_Write_Nibble(0x03, 0); // 8-bit mode (part 1)
    delay_ms(5);
    LCD_Write_Nibble(0x03, 0); // 8-bit mode (part 2)
    delay_ms(1);
    LCD_Write_Nibble(0x03, 0); // 8-bit mode (part 3)
    delay_ms(1);
    LCD_Write_Nibble(0x02, 0); // 4-bit mode
    delay_ms(1);

    // Function set: 4-bit mode, 2 lines, 5x8 font
    LCD_Write_Byte(LCD_FUNCTIONSET | LCD_2LINE | LCD_5x8DOTS, 0);
    delay_ms(1);

    // Display control: Display on, cursor off, blink off
    LCD_Write_Byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    delay_ms(1);

    // Clear display
    LCD_Clear();

    // Entry mode set: Left to right, no shift
    LCD_Write_Byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT, 0);
    delay_ms(1);
}

void LCD_Clear(void)
{
    // Only perform operations if LCD is powered
    if (!lcd_powered)
    {
        return;
    }

    LCD_Write_Byte(LCD_CLEARDISPLAY, 0);
    delay_ms(2); // Clear command needs longer delay
}

void LCD_Set_Cursor(uint8_t row, uint8_t col)
{
    // Only perform operations if LCD is powered
    if (!lcd_powered)
    {
        return;
    }

    uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    LCD_Write_Byte(LCD_SETDDRAMADDR | (col + row_offsets[row]), 0);
    delay_ms(1);
}

void LCD_Print(char *str)
{
    // Only perform operations if LCD is powered
    if (!lcd_powered)
    {
        return;
    }

    while (*str)
    {
        LCD_Write_Byte(*str++, 1);
    }
}

void LCD_Power_Off(void)
{
    if (lcd_powered)
    {
        // Turn off display first (proper shutdown)
        LCD_Write_Byte(LCD_DISPLAYCONTROL, 0); // Display off, cursor off, blink off
        delay_ms(1);

        // Turn off backlight
        uint8_t data = LCD_NOBACKLIGHT; // No backlight
        I2C0_SendByte(LCD_WRITE_ADDR, data);

        // Update power state flag
        lcd_powered = false;
    }
}

void LCD_Power_On(void)
{
    if (!lcd_powered)
    {
        // Set the power flag first
        lcd_powered = true;

        // Re-initialize the LCD from scratch
        LCD_Init();
    }
}

bool LCD_Is_Powered(void)
{
    return lcd_powered;
}