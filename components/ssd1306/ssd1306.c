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
#include <stdlib.h>
#include <malloc.h>

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp8266/rom_functions.h"
#include "task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "result_codes.h"
#include "i2c.h"
#include "ssd1306.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  SSD1306 CONTROL BYTE MACROS
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define SSD1306_COMMAND_SINGLE_BYTE 0x80
#define SSD1306_COMMAND_MULTI_BYTE  0x00
#define SSD1306_DATA_STREAM         0x40

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  SSD1306 COMMAND MACROS
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define SSD1306_SET_MEMORY_ADDRESSING_MODE          0x20
#define SSD1306_SET_COLUMN_ADDRESS                  0x21
#define SSD1306_SET_PAGE_ADDRESS                    0x22
#define SSD1306_RIGHT_HORIZONTAL_SCROLL             0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL              0x27
#define SSD1306_VERTICAL_RIGHT_HORIZONTAL_SCROLL    0x29
#define SSD1306_VERTICAL_LEFT_HORIZONTAL_SCROLL     0x2A
#define SSD1306_DEACTIVATE_SCROLL                   0x2E
#define SSD1306_ACTIVATE_SCROLL                     0x2F
#define SSD1306_SET_DISPLAY_START_LINE              0x40 
#define SSD1306_SET_CONTRAST                        0x81
#define SSD1306_SET_CHARGE_PUMP                     0x8D
#define SSD1306_SET_SEGMENT_REMAP_COL_TO_0          0xA0
#define SSD1306_SET_SEGMENT_REMAP_COL_TO_127        0xA1
#define SSD1306_SET_VERTICAL_SCROLL_AREA            0xA3
#define SSD1306_DISPLAY_ON_FOLLOW_RAM               0xA4
#define SSD1306_DISPLAY_ON_IGNORE_RAM               0xA5
#define SSD1306_SET_NORMAL_DISPLAY                  0xA6
#define SSD1306_SET_INVERSE_DISPLAY                 0xA7
#define SSD1306_SET_MULTIPLEX_RATIO                 0xA8
#define SSD1306_SET_DISPLAY_OFF                     0xAE
#define SSD1306_SET_DISPLAY_ON                      0xAF
#define SSD1306_SET_COM_OUTPUT_SCAN__NORMAL         0xC0
#define SSD1306_SET_COM_OUTPUT_SCAN_REMAPPED        0xC8
#define SSD1306_SET_DISPLAY_OFFSET                  0xD3
#define SSD1306_SET_DCLCK_DIV_RATION_FOSC           0xD5
#define SSD1306_SET_PRECHARGE_PERIOD                0xD9
#define SSD1306_SET_COM_PINS_HW_CONFIGURATION       0xDA
#define SSD1306_SET_VCOMH_DESELECT_LEVEL            0xDB
#define SSD1306_NOP                                 0xE3
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Following macro used to validate and assign context to ptr
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define ASSIGN_CONTEXT(ptr, context, func_name)                             \
if((context)==NULL) {                                                       \
    ESP_LOGE(SSD_TAG, "%s: context pointer cannot be NULL.", func_name);    \
    return INVALID_ARGUMENT;                                                \
}                                                                           \
if(((ssd1306_t *)(context))->header != (uint32_t)(context)) {               \
    ESP_LOGE(SSD_TAG, "%s: context pointer corrupt. %u != %u",              \
                func_name, (uint32_t)context,                               \
                ((ssd1306_t *)(context))->header);                          \
    return INVALID_ARGUMENT;                                                \
}                                                                           \
ptr = (ssd1306_t *)context
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Convenience macro to remove clutter 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define SSD1306_SEND_CMD(ptr, cmd, ret)                                     \
    ret = sendCommand(ptr, cmd);                                            \
    if (ret != OK) {                                                        \
        return ret;                                                         \
    }                                                                       
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Macros that define/configure the OLED display
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define SSD1306_PAGES                       8      
#define SSD1306_SEGMENTS                    128    // each segment is 8 bits tall
#define SSD1306_MIN_CONTRAST                0x00
#define SSD1306_MAX_CONTRAST                0xFF
#define SSD1306_HORIZONTAL_ADDRESSING_MODE  0x00
#define SSD1306_VERTICAL_ADDRESSING_MODE    0x01
#define SSD1306_PAGE_ADDRESSING_MODE        0x02
#define SSD1306_DEFAULT_ADDRESSING_MODE     SSD1306_HORIZONTAL_ADDRESSING_MODE

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * A PAGE consists of a vertical 8-bit column that spans the width of the
 * SSD1306 display. The following macros allow methods to pick out the
 * specific bit in the column to light up or shut off the bit.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define PAGE_TURN_COLUMN_ON  0xFF 
#define PAGE_TURN_COLUMN_OFF 0x00 

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define MIN(x, y)   (x) <= (y) ? (x) : (y)
#define MAX(x, y)   (x) >  (y) ? (x) : (y)

typedef struct _ssd1306_t {
    uint32_t    header;
    void        *i2c;
}ssd1306_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Private methods
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
static RESULT initializeDisplay(ssd1306_t *ptr, uint8_t contrast);
static RESULT sendCommand(ssd1306_t *ptr, uint8_t cmd);
static RESULT setWriteLocation(ssd1306_t *ptr, uint8_t scol, uint8_t ecol, uint8_t spage, uint8_t epage);
static RESULT drawPixel(ssd1306_t *ptr, uint8_t row, uint8_t col); 

static const char *SSD_TAG = "SSD1306";
static const uint8_t PIXEL_ON  = 1;
static const uint8_t PAGE_COLUMN_ON = PAGE_TURN_COLUMN_ON;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * Public method to initialize the SSD1306 display.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT 
SSD1306_Initialize(uint8_t slaveAddress, uint8_t scl, uint8_t sda, uint8_t contrast, void **context)
{
    #ifdef DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_Initialize(): Inputs slaveAddress: 0x%x, SCL:%d, SDA:%d.", slaveAddress, scl, sda);
    #endif

    if (context == NULL) {
        ESP_LOGE(SSD_TAG, "SSD1306_Initialize(): Context pointer cannot be null.");
        return INVALID_ARGUMENT;    
    }

    void *i2c = NULL;
    RESULT ret = I2C_Initialize(slaveAddress, scl, sda, true, &i2c);

    if (ret != OK) {
        ESP_LOGE(SSD_TAG, "SSD1306_Initialize(): Failed to initialize I2C. Error=%d.", ret);
        return ret; 
    }

    ssd1306_t *ptr = (ssd1306_t *)malloc(sizeof(ssd1306_t));
    if (ptr == NULL) {
        ESP_LOGE(SSD_TAG, "SSD1306_Initialize(): Failed to allocate memory for display context!");
        return FAILED_TO_ALLOCATE_MEMORY;    
    }

    ptr->header   = (uint32_t)ptr;
    ptr->i2c      = i2c;
    *context = (void *)ptr;

    ret = initializeDisplay(ptr, contrast);
    if (ret != OK) {
        ESP_LOGE(SSD_TAG, "SSD1306_Initialize(): Failed to initialize display. Error = %d.", ret);
    }

    return ret; 
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to turn free allocated context pointer. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT SSD1306_FreeContext(void **ppContext)
{
    ssd1306_t *ptr;
    
    if (ppContext == NULL) {
        return OK;
    }

    ASSIGN_CONTEXT(ptr, *ppContext, "SSD1306_FreeContext");

    RESULT ret = I2C_FreeContext(&ptr->i2c);

    if (ret != OK) {
        ESP_LOGE(SSD_TAG, "FreeContext(): Failed to free I2C context!");
        return ret;
    }

    free(ptr);
    *ppContext = NULL; 
    return ret;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to turn the display on. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_TurnDisplayOn(const void *context)
{
    #ifdef DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_TurnDisplayOn(): Turning display on."); 
    #endif

    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_DisplayOn");
    RESULT ret;
    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_SINGLE_BYTE, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_ON, ret);
    I2C_StopXmit(ptr->i2c);

    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to turn the display off.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_TurnDisplayOff(const void *context)
{
    #ifdef DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_TurnDisplayOff(): Turning display off."); 
    #endif

    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_DisplayOff");
    RESULT ret;
    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_SINGLE_BYTE, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_OFF, ret);
    I2C_StopXmit(ptr->i2c);

    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to fill display with a single character type. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_FillDisplay(const void *context, uint8_t fillChar)
{
    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_DisplayOn");

    #if DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_FillDisplay(): Filling display with fillChar 0x%x", fillChar);
    #endif

    RESULT ret = setWriteLocation(ptr, 0, SSD1306_WIDTH-1, 0, SSD1306_PAGES-1);
    if (ret != OK) {
        return ret;
    }

    for (int page=0; page<64; page++) {                         // each pass fills in a new page-worth of data  
        I2C_StartXmit(ptr->i2c);
        SSD1306_SEND_CMD(ptr, SSD1306_DATA_STREAM, ret);        
        for (int col=0; col<16; col++) {                        // each pass fills in 16x8=128 bits of info
            ret = I2C_Write(ptr->i2c, fillChar);
            if (ret != OK) {
                ESP_LOGE(SSD_TAG, "SSD1306_FillDisplay(): Failed to send data byte: 0x%x! Error = %d", fillChar, ret);
            }
        }
        I2C_StopXmit(ptr->i2c);
    }

    return OK; 
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to clear the display. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_ClearDisplay(const void *context)
{
    #if DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_ClearDisplay(): calling FillDisplay(0) to clear screen...");    
    #endif

    RESULT ret = SSD1306_FillDisplay(context, 0);
    if (ret != OK) {
        ESP_LOGE(SSD_TAG, "SSD1306_ClearDisplay(): Failed to clear display. Error = %d.", ret);
    }

    return ret;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to draw a single pixel on the screen.
 *
 * NOTE
 *  point.row must be less than SSD1306_HEIGHT
 *  point.col must be less than SSD1306_WIDTH
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_DrawPixel(const void *context, uint8_t row, uint8_t col) 
{
    if ((row >= SSD1306_HEIGHT) || (col >= SSD1306_WIDTH)) {
        return COORDINATE_OUT_OF_RANGE;
    }

    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_DrawPixel");

    return drawPixel(ptr, row, col);

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to draw a line from point p1 to point p2.
 *  
 *  NOTE
 *      Algo: calculate slope then light up points between the two points.
 *            row in [0, 64], col in [0, 128] inclusive
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_DrawLine(const void *context, const POINT p1, const POINT p2)
{
    if ((p2.col == p1.col) && (p2.row == p1.row)) {
        ESP_LOGE(SSD_TAG, "SSD1306_DrawLine(): Points are identical. No line possible.");
        return OK;  // technically this is not an error.
    }

    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_DrawPixel");

    // clip the points if they're out of range
    uint8_t r1 = MIN(p1.row, SSD1306_HEIGHT);
    uint8_t r2 = MIN(p2.row, SSD1306_HEIGHT);
    uint8_t c1 = MIN(p1.col, SSD1306_WIDTH);
    uint8_t c2 = MIN(p2.col, SSD1306_WIDTH);

    // identify extrema for points
    uint8_t minRow = MIN(r1, r2);
    uint8_t maxRow = MAX(r1, r2);
    uint8_t minCol = MIN(c1, c2);
    uint8_t maxCol = MAX(c1, c2);

    // identify affected pages
    uint8_t minPage = minRow/SSD1306_PAGES;
    uint8_t maxPage = maxRow/SSD1306_PAGES;

    if (r1 == r2) {
        RESULT ret = setWriteLocation(ptr, minCol, maxCol, minPage, maxPage); 
        if (ret != OK) {
            ESP_LOGE(SSD_TAG, "SSD1306_DrawLine(): Failed to set cursor position. p1=(%d, %d), p2=(%d, %d).",
                        p1.row, p1.col, p2.row, p2.col);
            return ret;
        }

        // horizontal line - slope is infinite
        I2C_StartXmit(ptr->i2c);
        SSD1306_SEND_CMD(ptr, SSD1306_DATA_STREAM, ret);        
        for (uint8_t x=minCol; x<maxCol; x++) { 
            I2C_Write(ptr->i2c, 1<<(r1-(8*minPage)));       // doesn't matter which page since r1==r2
        }
        I2C_StopXmit(ptr->i2c);

    } else if (c1 == c2) {
        RESULT ret = setWriteLocation(ptr, minCol, maxCol, minPage, maxPage); 
        if (ret != OK) {
            ESP_LOGE(SSD_TAG, "SSD1306_DrawLine(): Failed to set cursor position. p1=(%d, %d), p2=(%d, %d).",
                        p1.row, p1.col, p2.row, p2.col);
            return ret;
        }

        // vertical line - slope is zero
        uint8_t row = minRow, col_state, bit;;
        
        for (uint8_t x=minPage; x<maxPage+1; x++) {
            col_state = 0;
            bit = 0;
            I2C_StartXmit(ptr->i2c);
            SSD1306_SEND_CMD(ptr, SSD1306_DATA_STREAM, ret);
            // identify all the column bits that need to be turned on in the current page.
            while ((bit < 8) && (row <= maxRow)) {
                // this bit needs to be turned on if it lies within the range of the current row values.
                if (row == ((8*x)+bit)) {
                    col_state |= 1<<(row-(8*x));
                    row++;  // move on to next row bit
                }
                bit++;
            } 
            I2C_Write(ptr->i2c, col_state);
            I2C_StopXmit(ptr->i2c);
        }

    } else { 
        // transverse line
        // (y - y1) = m(x2 - x1) --> y = mx + b
        // m = slope (y2-y1/x2-x1)
        // b = y1 - m(x1)
        double m = (double)(r2-r1)/(c2-c1);
        double b = (double)r1 - (m * c1);
        drawPixel(ptr, minRow, minCol);
        while (minCol < maxCol) {
            minRow = (m * minCol) + b;
            drawPixel(ptr, minRow, minCol);
            minCol++;
        }
        drawPixel(ptr, maxRow, maxCol);  
    }

    return NOT_IMPLEMENTED;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_DrawRectangle(const void *context, const POINT p1, const POINT p2, const POINT p3, const POINT p4) 
{
    return NOT_IMPLEMENTED;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_DrawCircle(const void *context, const POINT center, uint8_t radius)
{
    return NOT_IMPLEMENTED;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to update one of the device's PAGEs. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
SSD1306_UpdatePage(const void *context, uint8_t pageId, const PAGE *page) 
{
    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_UpdatePage");

    #if DEBUG
    ESP_LOGD(SSD_TAG, "SSD1306_UpdatePage(): called to update PAGE[%d].", pageId);
    #endif

    if (pageId > (SSD1306_PAGES-1)) {
        ESP_LOGE(SSD_TAG, "SSD1306_UpdatePage(): pageId (%d) cannot be greater than %d.", pageId, SSD1306_PAGES-1);
        return INVALID_ARGUMENT; 
    }

    RESULT ret = setWriteLocation(ptr, 0, SSD1306_WIDTH-1, pageId, pageId);
    if (ret != OK) {
        return ret;
    }

    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_DATA_STREAM, ret);            
    for (int col=0; col<16; col++) {                     
        ret = I2C_Write(ptr->i2c, page->page[col]);
        if (ret != OK) {
            ESP_LOGE(SSD_TAG, "SSD1306_UpdatePage(): Failed to send data byte: 0x%x! Error = %d", page->page[col], ret);
        }
    }
    I2C_StopXmit(ptr->i2c);
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method to set the contrast level.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT SSD1306_SetContrast(const void *context, uint8_t contrast)
{
    ssd1306_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "SSD1306_SetContrast");
    
    RESULT ret;
    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_MULTI_BYTE, ret);     
    SSD1306_SEND_CMD(ptr, SSD1306_SET_CONTRAST, ret);
    SSD1306_SEND_CMD(ptr, contrast, ret);
    I2C_StopXmit(ptr->i2c);
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * Private method to configure the ssd1306 display.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT 
initializeDisplay(ssd1306_t *ptr, uint8_t contrast)
{
    #ifdef DEBUG
    ESP_LOGD(SSD_TAG, "initializeDisplay(): called to configure display.");
    #endif

    RESULT ret;
    I2C_StartXmit(ptr->i2c);

    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_MULTI_BYTE, ret);             
    SSD1306_SEND_CMD(ptr, SSD1306_SET_MULTIPLEX_RATIO, ret);
    SSD1306_SEND_CMD(ptr, 0x3F, ret);                                   // RESET value: 63 maps to 64MUX
    SSD1306_SEND_CMD(ptr, SSD1306_SET_MEMORY_ADDRESSING_MODE, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_DEFAULT_ADDRESSING_MODE, ret);           
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_OFFSET, ret);
    SSD1306_SEND_CMD(ptr, 0x00, ret);                                   // display offset
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_START_LINE, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_SEGMENT_REMAP_COL_TO_127, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_COM_OUTPUT_SCAN__NORMAL, ret);    // Set COM output scan direction
    SSD1306_SEND_CMD(ptr, SSD1306_SET_COM_PINS_HW_CONFIGURATION, ret);
    SSD1306_SEND_CMD(ptr, 0x12, ret);                                   // COM Hardware Configuration (128x64)
    SSD1306_SEND_CMD(ptr, SSD1306_SET_CONTRAST, ret);
    SSD1306_SEND_CMD(ptr, contrast, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_DISPLAY_ON_FOLLOW_RAM, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_NORMAL_DISPLAY, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DCLCK_DIV_RATION_FOSC, ret);
    SSD1306_SEND_CMD(ptr, 0x80, ret);                                   // RESET values: divide ratio=1, Fosc=8
    SSD1306_SEND_CMD(ptr, SSD1306_SET_CHARGE_PUMP, ret);
    SSD1306_SEND_CMD(ptr, 0x14, ret);                                   // Internal DC/DC
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_ON, ret);

#if 0
    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_MULTI_BYTE, ret);             // send control byte - command stream
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_OFF, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DCLCK_DIV_RATION_FOSC, ret);
    SSD1306_SEND_CMD(ptr, 0x80, ret);                                   // RESET values: divide ratio=1, Fosc=8
    SSD1306_SEND_CMD(ptr, SSD1306_SET_MULTIPLEX_RATIO, ret);
    SSD1306_SEND_CMD(ptr, 0x3F, ret);                                   // RESET value: 63 maps to 64MUX
    SSD1306_SEND_CMD(ptr, SSD1306_SET_MEMORY_ADDRESSING_MODE, ret);
    SSD1306_SEND_CMD(ptr, DEFAULT_ADDRESSING_MODE_HORIZONTAL, ret);           
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_OFFSET, ret);
    SSD1306_SEND_CMD(ptr, 0x00, ret);                                   // display offset
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_START_LINE, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_CHARGE_PUMP, ret);
    SSD1306_SEND_CMD(ptr, 0x14, ret);                                   // Internal DC/DC
    SSD1306_SEND_CMD(ptr, SSD1306_SET_SEGMENT_REMAP_COL_TO_0, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_COM_OUTPUT_SCAN__NORMAL, ret);    // Set COM output scan direction
    SSD1306_SEND_CMD(ptr, SSD1306_SET_COM_PINS_HW_CONFIGURATION, ret);
    SSD1306_SEND_CMD(ptr, 0x12, ret);                                   // COM Hardware Configuration (128x64)
    SSD1306_SEND_CMD(ptr, SSD1306_SET_CONTRAST, ret);
    SSD1306_SEND_CMD(ptr, ptr->contrast, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_PRECHARGE_PERIOD, ret);
    SSD1306_SEND_CMD(ptr, 0xF1, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_DISPLAY_ON_FOLLOW_RAM, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_NORMAL_DISPLAY, ret);
    SSD1306_SEND_CMD(ptr, SSD1306_SET_DISPLAY_ON, ret);
#endif

    I2C_StopXmit(ptr->i2c);
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Private method to send a command to the ssd1306 display. 
 *
 * Commands require Continuation bit to be set to 1.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static inline 
RESULT sendCommand(ssd1306_t *ptr, uint8_t cmd) 
{
    RESULT ret = I2C_Write(ptr->i2c, cmd);
    if (ret != OK) {
        ESP_LOGE(SSD_TAG, "sendCommand(): Failed to send command: 0x%x! Error = %d", cmd, ret);
    }

    return ret;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Private method to set cursor position prior to write. 
 *
 *  NOTE
 *      This method assums Column Addressing scheme.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT 
setWriteLocation(ssd1306_t *ptr, uint8_t scol, uint8_t ecol, uint8_t spage, uint8_t epage)
{
    RESULT ret;
    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_COMMAND_MULTI_BYTE, ret);      
    SSD1306_SEND_CMD(ptr, SSD1306_SET_COLUMN_ADDRESS, ret);
    SSD1306_SEND_CMD(ptr, scol, ret); 
    SSD1306_SEND_CMD(ptr, ecol, ret); 
    SSD1306_SEND_CMD(ptr, SSD1306_SET_PAGE_ADDRESS, ret);
    SSD1306_SEND_CMD(ptr, spage, ret); 
    SSD1306_SEND_CMD(ptr, epage, ret); 
    I2C_StopXmit(ptr->i2c);
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
 * Private method to draw a pixel on the screen.  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT 
drawPixel(ssd1306_t *ptr, uint8_t row, uint8_t col)
{
    uint8_t pageId = row/SSD1306_PAGES;

    RESULT ret = setWriteLocation(ptr, col, col, pageId, pageId);
    if (ret != OK) {
        return ret;
    }

    // NOTE: Need to light up the appropriate column-bit. 
    //       1 << (row - (8 * pageId)
    //       e.g. row = 30 (pageId=30/8=3)
    //       light up column 7th bit from bottom. 
    //       1 << (30 - 24) => 1 << 6 = 0b01000000
    I2C_StartXmit(ptr->i2c);
    SSD1306_SEND_CMD(ptr, SSD1306_DATA_STREAM, ret);            
    I2C_Write(ptr->i2c, 1<<(row-(8*pageId)));
    I2C_StopXmit(ptr->i2c);
    return OK;
}
