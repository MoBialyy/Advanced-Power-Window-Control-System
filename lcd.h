#ifndef LCD_H
#define LCD_H
#include <stdint.h>



static void I2C0_Init(void);
static void I2C0_WriteByte(uint8_t addr, uint8_t data);
static void LCD_Write4Bits(uint8_t data);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);
void LCD_PrintNum(uint32_t num);
void LCD_SetBacklight(uint8_t on);


#endif
