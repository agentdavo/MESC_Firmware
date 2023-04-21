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

/**
 * This Library is written and optimized by Olivier Van den Eede(4ilo) in 2016
 * for Stm32 Uc and HAL-i2c lib's.
 *
 * To use this library with ssd1306 oled display you will need to customize the defines below.
 *
 * This library uses 2 extra files (fonts.c/h).
 * In this files are 3 different fonts you can use:
 * 		- Font_7x10
 * 		- Font_11x18
 * 		- Font_16x26
 *
 */
#pragma once

#include <fonts.h>

#include <stdint.h>
#include <stdbool.h>

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR 0x78 ///< I2c address
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH 128 ///< SSD1306 width in pixels
#endif


#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT 64 ///< SSD1306 LCD height in pixels
#endif

/**
 * Enumeration for screen colors
 */
typedef enum
{
	Black = 0x00, ///< Black color, no pixel
	White = 0x01  ///< Pixel is set. Color depends on LCD
} SSD1306_COLOR;

/**
 * Struct to store transformations.
 */
typedef struct
{
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

#ifdef __cplus_plus
extern "C"
{
#endif

  /**
 * Screen initialization.
 * @return true - success, false failed.
 */
bool ssd1306_Init();

/**
 * Fill the whole screen with the given color.
 * @param color
 */
void ssd1306_Fill(SSD1306_COLOR color);

/**
 * Draw one pixel in the screenbuffer.
 * @param x coordinate.
 * @param y coordinate.
 * @param color pixel color.
 */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);

/**
 * Write the screenbuffer with changed to the screen.
 */
void ssd1306_UpdateScreen();

/**
 * Draw 1 char to the screen buffer.
 * @param ch character which needed to be written.
 * @param Font selected font.
 * @param color black or white.
 * @return written char for validation
 */
char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color);

/**
 * Write full string to screenbuffer.
 * @param str
 * @param Font
 * @param color
 * @return '\0' if ok, any character from str if problems occurred.
 */
char ssd1306_WriteString(char const *str, FontDef Font, SSD1306_COLOR color);

/**
* Invert background/foreground colors.
*/
void ssd1306_InvertColors(void);

/**
 * Set cursor position.
 * @param x position.
 * @param y position.
 */
void ssd1306_SetCursor(uint8_t x, uint8_t y);

#ifdef __cplus_plus
}
#endif
