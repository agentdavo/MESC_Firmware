/*
Copyright (c) 2016-2021 Olivier Van den Eede - ovde.be

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/
#include <ssd1306.h>

#include <HAL/MESC_HAL.h>

#include <stdint.h>

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];  ///< Screenbuffer

static SSD1306_t SSD1306;  ///< Screen object

bool ssd1306_Init()
{
  // Wait for the screen to boot
  Delay(100);
  int status = 0;

  // Init LCD
  status += ssd1306_WriteCommand(0xAE);  // Display off
  status += ssd1306_WriteCommand(0x20);  // Set Memory Addressing Mode
  status += ssd1306_WriteCommand(0x10);  // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  status += ssd1306_WriteCommand(0xB0);  // Set Page Start Address for Page Addressing Mode,0-7
  status += ssd1306_WriteCommand(0xC8);  // Set COM Output Scan Direction
  status += ssd1306_WriteCommand(0x00);  // Set low column address
  status += ssd1306_WriteCommand(0x10);  // Set high column address
  status += ssd1306_WriteCommand(0x40);  // Set start line address
  status += ssd1306_WriteCommand(0x81);  // set contrast control register
  status += ssd1306_WriteCommand(0xFF);
  status += ssd1306_WriteCommand(0xA1);  // Set segment re-map 0 to 127
  status += ssd1306_WriteCommand(0xA6);  // Set normal display

  status += ssd1306_WriteCommand(0xA8);  // Set multiplex ratio(1 to 64)
  status += ssd1306_WriteCommand(SSD1306_HEIGHT - 1);

  status += ssd1306_WriteCommand(0xA4);  // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  status += ssd1306_WriteCommand(0xD3);  // Set display offset
  status += ssd1306_WriteCommand(0x00);  // No offset
  status += ssd1306_WriteCommand(0xD5);  // Set display clock divide ratio/oscillator frequency
  status += ssd1306_WriteCommand(0xF0);  // Set divide ratio
  status += ssd1306_WriteCommand(0xD9);  // Set pre-charge period
  status += ssd1306_WriteCommand(0x22);

  status += ssd1306_WriteCommand(0xDA);  // Set com pins hardware configuration
#ifdef SSD1306_COM_LR_REMAP
  status += ssd1306_WriteCommand(0x32);  // Enable COM left/right remap
#else
  status += ssd1306_WriteCommand(0x12);  // Do not use COM left/right remap
#endif  // SSD1306_COM_LR_REMAP

  status += ssd1306_WriteCommand(0xDB);  // Set vcomh
  status += ssd1306_WriteCommand(0x20);  // 0x20,0.77xVcc
  status += ssd1306_WriteCommand(0x8D);  // Set DC-DC enable
  status += ssd1306_WriteCommand(0x14);  //
  status += ssd1306_WriteCommand(0xAF);  // Turn on SSD1306 panel

  if (status != 0)
    {
      return 1;
    }

  // Clear screen
  ssd1306_Fill(Black);

  // Flush buffer to screen
  ssd1306_UpdateScreen();

  // Set default values for screen object
  SSD1306.CurrentX = 0;
  SSD1306.CurrentY = 0;

  SSD1306.Initialized = 1;

  return 0;
}

void ssd1306_Fill(SSD1306_COLOR color)
{
  for (uint32_t i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
      SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

void ssd1306_UpdateScreen()
{
  for (uint8_t i = 0; i < 8; i++)
    {
      ssd1306_WriteCommand(0xB0 + i);
      ssd1306_WriteCommand(0x00);
      ssd1306_WriteCommand(0x10);

      ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
      //OI HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}

void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
      // Don't write outside the buffer
      return;
    }

  // Check if pixel should be inverted
  if (SSD1306.Inverted)
    {
      color = (SSD1306_COLOR)!color;
    }

  // Draw in the right color
  if (color == White)
    {
      SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
  else
    {
      SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
  // Check remaining space on current line
  if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) || SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
    {
      // Not enough space on current line
      return 0;
    }

  for (uint32_t i = 0; i < Font.FontHeight; i++)
    {
      uint32_t b = Font.data[(ch - 32) * Font.FontHeight + i];
      for (uint32_t j = 0; j < Font.FontWidth; j++)
        {
          if ((b << j) & 0x8000)
            {
              ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
            }
          else
            {
              ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

  // The current space is now taken
  SSD1306.CurrentX += Font.FontWidth;

  return ch;
}

char ssd1306_WriteString(char const* str, FontDef Font, SSD1306_COLOR color)
{
  while (*str)
    {
      if (ssd1306_WriteChar(*str, Font, color) != *str)
        {
          return *str;  // Char could not be written
        }
      str++;
    }
  return '\0';
}

void ssd1306_InvertColors(void)
{
  SSD1306.Inverted = !SSD1306.Inverted;
}

void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
  SSD1306.CurrentX = x;
  SSD1306.CurrentY = y;
}
