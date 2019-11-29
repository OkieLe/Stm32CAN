#ifndef ___LCD_ST7789_H
#define ___LCD_ST7789_H

//choose another MCU series if you are not using STM32F4.
#include "stm32h7xx_hal.h"

//if you predefined pin names, you could find them in main.h and use them below
#include "main.h"

/****************
 * Comment one to use another one.
 * two parameters can be choosed
 * 135x240(0.96 inch) and 240x240(1.3inch)
 * X_SHIFT&Y_SHIFT are used to correct different display's resolution
 * USING_HORIZONAL |  0  &  1  |  2  &  3  |
 * Display rotation| Vertical  | Horizonal |
 */

//#define USING_135X240
#define USING_240X240

#ifdef USING_135X240
#define USING_HORIZONAL 2

#if USING_HORIZONAL == 0
#define ST7789_WIDTH 135
#define ST7789_HEIGHT 240
#define X_SHIFT 52
#define Y_SHIFT 40
#endif

#if USING_HORIZONAL == 1
#define ST7789_WIDTH 135
#define ST7789_HEIGHT 240
#define X_SHIFT 52
#define Y_SHIFT 40
#endif

#if USING_HORIZONAL == 2
#define ST7789_WIDTH 240
#define ST7789_HEIGHT 135
#define X_SHIFT 40
#define Y_SHIFT 53
#endif

#if USING_HORIZONAL == 3
#define ST7789_WIDTH 240
#define ST7789_HEIGHT 135
#define X_SHIFT 40
#define Y_SHIFT 52
#endif

#endif

#ifdef USING_240X240

#define USING_HORIZONAL 2
#define ST7789_WIDTH 240
#define ST7789_HEIGHT 240
#define X_SHIFT 0
#define Y_SHIFT 0

#endif

#define ABS(x) ((x) > 0 ? (x) : -(x))

#define ST7789_CMD 0  //Write command index
#define ST7789_DATA 1 //Write data index

//basic functions.
void ST7789_Init(SPI_HandleTypeDef *hspi);
void ST7789_SetBrightness(uint8_t Brightness);
void ST7789_Fill_Color(uint16_t color);
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);

//Graphical functions.
void ST7789_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7789_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

//Extented Graphical functions.
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);

//Color of pen
//If you want to use another color, you can choose one in RGB format.
#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define BROWN 0XBC40
#define BRRED 0XFC07
#define GRAY 0X8430

#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458
//PANEL COLOR

#define LIGHTGREEN 0X841F
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12

#endif
