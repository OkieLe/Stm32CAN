#include "lcd_st7789.h"

uint16_t BACK_COLOR = 0x0000; //User customized background color

SPI_HandleTypeDef* hspi;

/**
 * @brief Select ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Select()
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Unselect the ST7789 controller
 * @param none
 * @return none
 */
void ST7789_UnSelect()
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Reset the ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Reset()
{
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
}

/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
void ST7789_WriteCommand(uint8_t cmd)
{
  ST7789_Select();
  uint16_t TxData = cmd;
  HAL_SPI_Transmit(hspi, (uint8_t*)&TxData, 1, HAL_MAX_DELAY);
  ST7789_UnSelect();
}

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
static void ST7789_WriteData(uint8_t *buff, size_t buff_size)
{
  ST7789_Select();
  uint16_t index = 0;
  while (buff_size > 0 && index < buff_size)
  {
    uint16_t TxData =  buff[index] | 0x100;
    HAL_SPI_Transmit(hspi, (uint8_t*)&TxData, 1, HAL_MAX_DELAY);
    index ++;
  }
  ST7789_UnSelect();
}
/**
 * @brief Write data to ST7789 controller, simplify for 8bit data.
 * data -> data to write
 * @return none
 */
static void ST7789_WriteData_Fixed(uint8_t data)
{
  ST7789_Select();
  uint16_t TxData = data | 0x100;
  HAL_SPI_Transmit(hspi, (uint8_t*)&TxData, 1, HAL_MAX_DELAY);
  ST7789_UnSelect();
}
/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  ST7789_Select();

  //CASET
  ST7789_WriteCommand(0x2A);
  {
    uint8_t data[] = {((x0 + X_SHIFT) >> 8) & 0xFF, (x0 + X_SHIFT) & 0xFF, ((x1 + X_SHIFT) >> 8) & 0xFF, (x1 + X_SHIFT) & 0xFF};
    ST7789_WriteData(data, sizeof(data));
  }

  //RASET
  ST7789_WriteCommand(0x2B);
  {
    uint8_t data[] = {((y0 + Y_SHIFT) >> 8) & 0xFF, (y0 + Y_SHIFT) & 0xFF, ((y1 + Y_SHIFT) >> 8) & 0xFF, (y1 + Y_SHIFT) & 0xFF};
    ST7789_WriteData(data, sizeof(data));
  }
  //write to RAM
  //RAMWR
  ST7789_WriteCommand(0x2C);
  ST7789_UnSelect();
}

void ST7789_SetBrightness(uint8_t Brightness)
{
  ST7789_WriteCommand(0x53);
  ST7789_WriteData_Fixed(0x14);
  ST7789_WriteCommand(0x51);
  ST7789_WriteData_Fixed(Brightness);
}

void ST7789_Init(SPI_HandleTypeDef *spi)
{
  hspi = spi;
  ST7789_Select();
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  HAL_Delay(300);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  ST7789_WriteCommand(0x36);

  switch (USING_HORIZONAL)
  {
    case 0 : ST7789_WriteData_Fixed(0x00); break;
    case 1 : ST7789_WriteData_Fixed(0xC0); break;
    case 2 : ST7789_WriteData_Fixed(0x70); break;
    case 3 : ST7789_WriteData_Fixed(0xA0); break;
    default : break;
  }

  ST7789_WriteCommand(0x3A);
  ST7789_WriteData_Fixed(0x05);
  ST7789_WriteCommand(0xB2);
  {
    uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
    ST7789_WriteData(data, sizeof(data));
  }
  ST7789_WriteCommand(0XB7);
  ST7789_WriteData_Fixed(0x35);
  ST7789_WriteCommand(0xBB);
  ST7789_WriteData_Fixed(0x19);
  ST7789_WriteCommand(0xC0);
  ST7789_WriteData_Fixed(0x2C);
  ST7789_WriteCommand(0xC2);
  ST7789_WriteData_Fixed(0x01);
  ST7789_WriteCommand(0xC3);
  ST7789_WriteData_Fixed(0x12);
  ST7789_WriteCommand(0xC4);
  ST7789_WriteData_Fixed(0x20);
  ST7789_WriteCommand(0xC6);
  ST7789_WriteData_Fixed(0x0F);
  ST7789_WriteCommand(0xD0);
  ST7789_WriteData_Fixed(0xA4);
  ST7789_WriteData_Fixed(0xA1);
  ST7789_WriteCommand(0xE0);
  {
    uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
    ST7789_WriteData(data, sizeof(data));
  }
    ST7789_WriteCommand(0xE1);
  {
    uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
    ST7789_WriteData(data, sizeof(data));
  }
  ST7789_WriteCommand(0x21);
  ST7789_WriteCommand(0x11);
  ST7789_WriteCommand(0x29);
  ST7789_UnSelect();
}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(uint16_t color)
{
  ST7789_Select();
  uint16_t i, j;
  ST7789_SetAddressWindow(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);
  for (i = 0; i < ST7789_WIDTH; i++)
    for (j = 0; j < ST7789_HEIGHT; j++)
    {
      uint8_t data[] = {color >> 8, color & 0xFF};
      ST7789_WriteData(data, sizeof(data));
    }
  ST7789_UnSelect();
}

/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  ST7789_Select();
  ST7789_SetAddressWindow(x, y, x, y);
  uint8_t data[] = {color >> 8, color & 0xFF};
  ST7789_WriteData(data, sizeof(data));
  ST7789_UnSelect();
}

/**
 * @brief Fill an Area with single color
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
  ST7789_Select();
  uint16_t i, j;
  ST7789_SetAddressWindow(xSta, ySta, xEnd, yEnd);
  for (i = ySta; i <= yEnd; i++)
    for (j = xSta; j <= xEnd; j++)
    {
      uint8_t data[] = {color >> 8, color & 0xFF};
      ST7789_WriteData(data, sizeof(data));
    }
  ST7789_UnSelect();
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  ST7789_Select();
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;

  delta_x = x2 - x1;
  delta_y = y2 - y1;

  uRow = x1;
  uCol = y1;

  if (delta_x > 0)
    incx = 1;
  else if (delta_x == 0)
    incx = 0;
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }

  if (delta_y > 0)
    incy = 1;
  else if (delta_y == 0)
    incy = 0;
  else
  {
    incy = -1;
    delta_y = -delta_x;
  }
  if (delta_x > delta_y)
    distance = delta_x;
  else
    distance = delta_y;

  for (t = 0; t <= distance; t++)
  {
    ST7789_DrawPixel(uRow, uCol, color);
    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if (yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
  ST7789_UnSelect();
}

/**
 * @brief Draw a Rectangle with single color
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param color -> color of the Rectangle line
 * @return none
 */
void ST7789_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  ST7789_Select();
  ST7789_DrawLine(x1, y1, x2, y1, color);
  ST7789_DrawLine(x1, y1, x1, y2, color);
  ST7789_DrawLine(x1, y2, x2, y2, color);
  ST7789_DrawLine(x2, y1, x2, y2, color);
  ST7789_UnSelect();
}

/**
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void ST7789_Draw_Circle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
  ST7789_Select();
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  ST7789_DrawPixel(x0, y0 + r, color);
  ST7789_DrawPixel(x0, y0 - r, color);
  ST7789_DrawPixel(x0 + r, y0, color);
  ST7789_DrawPixel(x0 - r, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ST7789_DrawPixel(x0 + x, y0 + y, color);
    ST7789_DrawPixel(x0 - x, y0 + y, color);
    ST7789_DrawPixel(x0 + x, y0 - y, color);
    ST7789_DrawPixel(x0 - x, y0 - y, color);

    ST7789_DrawPixel(x0 + y, y0 + x, color);
    ST7789_DrawPixel(x0 - y, y0 + x, color);
    ST7789_DrawPixel(x0 + y, y0 - x, color);
    ST7789_DrawPixel(x0 - y, y0 - x, color);
  }
  ST7789_UnSelect();
}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
  if ((x >= ST7789_WIDTH) || (y >= ST7789_HEIGHT))
    return;
  if ((x + w - 1) >= ST7789_WIDTH)
    return;
  if ((y + h - 1) >= ST7789_HEIGHT)
    return;

  ST7789_Select();
  ST7789_SetAddressWindow(x, y, x + w - 1, y + h - 1);
  ST7789_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
  ST7789_UnSelect();
}

/**
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void ST7789_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  ST7789_Select();
  uint8_t i;

  /* Check input parameters */
  if (
    x >= ST7789_WIDTH ||
    y >= ST7789_HEIGHT)
  {
    /* Return error */
    return;
  }

  /* Check width and height */
  if ((x + w) >= ST7789_WIDTH)
  {
    w = ST7789_WIDTH - x;
  }
  if ((y + h) >= ST7789_HEIGHT)
  {
    h = ST7789_HEIGHT - y;
  }

  /* Draw lines */
  for (i = 0; i <= h; i++)
  {
    /* Draw lines */
    ST7789_DrawLine(x, y + i, x + w, y + i, color);
  }
  ST7789_UnSelect();
}

/**
 * @brief Draw a Filled circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle
 * @return  none
 */
void ST7789_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
  ST7789_Select();
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  ST7789_DrawPixel(x0, y0 + r, color);
  ST7789_DrawPixel(x0, y0 - r, color);
  ST7789_DrawPixel(x0 + r, y0, color);
  ST7789_DrawPixel(x0 - r, y0, color);
  ST7789_DrawLine(x0 - r, y0, x0 + r, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ST7789_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
    ST7789_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

    ST7789_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
    ST7789_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
  }
  ST7789_UnSelect();
}
