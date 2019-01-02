/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 by Hector Cura Jr.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef _ssd1306_h__
#define _ssd1306_h__

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64

typedef struct _PAGE {
    uint8_t page[16];
}PAGE;

typedef struct _POINT {
    uint8_t row;
    uint8_t col;
}POINT;

RESULT SSD1306_Initialize(uint8_t slaveAddress, uint8_t scl, uint8_t sda, uint8_t contrast, void **context);
RESULT SSD1306_FreeContext(void **ppContext);
RESULT SSD1306_TurnDisplayOn(const void *context);
RESULT SSD1306_TurnDisplayOff(const void *context);
RESULT SSD1306_FillDisplay(const void *context, uint8_t fillChar);
RESULT SSD1306_ClearDisplay(const void *context);
RESULT SSD1306_DrawPixel(const void *context, uint8_t row, uint8_t col); 
RESULT SSD1306_DrawLine(const void *context, const POINT p1, const POINT p2);
RESULT SSD1306_DrawCircle(const void *context, const POINT center, uint8_t radius);
RESULT SSD1306_DrawRectangle(const void *context, const POINT p1, const POINT p2, const POINT p3, const POINT p4);
RESULT SSD1306_SetContrast(const void *context, uint8_t contrast); 
RESULT SSD1306_UpdatePage(const void *context, uint8_t pageId, const PAGE *page);
#endif // __ssd1306_h__ 

