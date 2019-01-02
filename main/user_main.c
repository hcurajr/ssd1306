#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "result_codes.h"
#include "ssd1306.h"

#define WIFI_NO_WIFI        0
#define WIFI_POOR_WIFI      1
#define WIFI_GOOD_WIFI      2
#define WIFI_EXCELLENT_WIFI 3

const uint8_t wifi_status[4][16] = {
    {0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},     // no wifi
    {0x00, 0x00, 0x02, 0x04, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},     // poor wifi
    {0x00, 0x08, 0x12, 0x14, 0x14, 0x12, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},     // good wifi
    {0x20, 0x48, 0x52, 0x54, 0x54, 0x52, 0x48, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}      // excellent wifi
};

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Program: ssd1306 
 *  Date   : 10/18/2018
 *  Author : hcurajr@hotmail.com
 *
 *  Description
 *  Add support for I2C protocol and SSD1306 display driver.
 *  The main file is the test file to confirm the I2C protocol and SSD1306 drivers work.
 *
 *  NOTES
 *      gpioX: GND - Ground
 *      gpioY: VCC - 3.3V input
 *      gpioA: SCL - Clock signal line
 *      gpioB: SDA - Data line
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#define SLAVE_ADDRESS 0x3C
#define SCL_PIN 4 
#define SDA_PIN 5 
#define DEFAULT_CONTRAST 255 

const char *getResultString(RESULT result) 
{
    switch (result) {
        case COORDINATE_OUT_OF_RANGE:
            return "Coordinate out of range.";
        case FAILED_READ:
            return "Read operation failed.";
        case FAILED_TO_ALLOCATE_MEMORY:
            return "Failed to allocate memory.";
        case FAILED_TO_CONFIGURE_I2C_PINS:
            return "Failed to configure I2C pins.";
        case FAILED_TO_INSTALL_ISR_FUNCTION:
            return "Failed to install ISR.";
        case FAILED_TO_INSTALL_ISR_SERVICE:
            return "Failed to install ISR Service.";
        case FAILED_TO_SET_INTERRUPT_TYPE:
            return "Failed to set Interrupt trigger type .";
        case FAILED_TO_SET_PIN_LEVEL:
            return "Failed to set Pin level.";
        case FAILED_TO_SEND_SLAVE_ADDRESS:
            return "Slave Address not ACK'd by slave device.";
        case FAILED_WRITE_RECEIVED_NACK:
            return "Received NACK from slave device.";
        case INVALID_ARGUMENT:
            return "Invalid Argument passed in to function.";
        case INVALID_CONTEXT:
            return "Invalid Context pointer.";
        case INVALID_SCL_PIN:
            return "SCL pin not valid.";
        case INVALID_SDA_PIN:
            return "SDA pin not valid.";
        case INVALID_SLAVE_ADDRESS_GT7F:
            return "Slave address cannot be greater than 0x7F.";
        case INVALID_SLAVE_ADDRESS_RESERVED:
            return "Slave address must not be a reserved value.";
        case INVALID_STATE_CHANGE_REQUEST:
            return "Invalid State Change Requested.";
        case NOT_IMPLEMENTED:
            return "Function Not Implemented.";
        case OK:
            return "OK";
        default:
            return "UNKNOWN ERROR!";
    }
}

void 
app_main(void)
{
    void *ssd1306 = NULL;
    ESP_LOGD("MAIN", "Calling SSD1306_initialize(0x%x, SCL:%d, SDA:%d, CONTRAST:0x%x).", SLAVE_ADDRESS, SCL_PIN, SDA_PIN, DEFAULT_CONTRAST);
    RESULT result = SSD1306_Initialize(SLAVE_ADDRESS, SCL_PIN, SDA_PIN, DEFAULT_CONTRAST, &ssd1306);
    if (result != OK) {
        ESP_LOGE("MAIN", "Initialization FAILED! Error: call to SSD1306_initialize returned: %s", getResultString(result));
        return;
    }  

    ESP_LOGD("MAIN", "\n---Test: Free SSD1306 Pointer---");
    RESULT ret = SSD1306_FreeContext(ssd1306);
    if (ret != OK) {
        ESP_LOGE("MAIN", "Test FAILED! Error = %d", ret);
        return;
    }
   
    result = SSD1306_Initialize(SLAVE_ADDRESS, SCL_PIN, SDA_PIN, DEFAULT_CONTRAST, &ssd1306);
    if (result != OK) {
        ESP_LOGE("MAIN", "Initialization FAILED! Error: call to SSD1306_initialize returned: %s", getResultString(result));
        return;
    }  

    ESP_LOGD("MAIN", "---Test: Clear the display---"); 
    SSD1306_ClearDisplay(ssd1306);
    vTaskDelay(25);
   
    for (uint8_t x=0; x<10; x++) {
        uint8_t fill = esp_random() % 256; 
        ESP_LOGD("MAIN", "\n---Test: Fill screen with '0x%x'---", fill);
        SSD1306_FillDisplay(ssd1306, fill);
        vTaskDelay(25); 
    }

    SSD1306_FillDisplay(ssd1306, 0xFF);

    ESP_LOGD("MAIN", "\n---Test: Decrease contrast---");
    for (uint8_t x=255; x>0; x-=5) {
        SSD1306_SetContrast(ssd1306, x);
        vTaskDelay(3);
    }
    vTaskDelay(25);

    ESP_LOGD("MAIN", "\n---Test: Increase contrast---");
    for (uint8_t x=0; x<255; x+=5) {
        SSD1306_SetContrast(ssd1306, x);
        vTaskDelay(3);
    }

    SSD1306_ClearDisplay(ssd1306);
    vTaskDelay(25);

    ESP_LOGD("MAIN", "\n---Test: Write PAGES to screen");
    for (int x=0; x<4; x++) {
        SSD1306_UpdatePage(ssd1306, 2*x, (const PAGE *)&wifi_status[x]);
        vTaskDelay(5);
    }

    vTaskDelay(50);
    SSD1306_ClearDisplay(ssd1306);

    while(1) {
        ESP_LOGD("MAIN", "\n---Test: Draw random pixels on screen---");
        for (int x=0; x<1000; x++) {
            uint8_t row, col;
            row = esp_random() % SSD1306_HEIGHT;
            col = esp_random() % SSD1306_WIDTH;
            SSD1306_DrawPixel(ssd1306, row, col);
            vTaskDelay(1);
        }

        vTaskDelay(50);
        SSD1306_ClearDisplay(ssd1306);

        ESP_LOGD("MAIN", "\n---Test: Draw Random lines---");
        for (int x=0; x<1000;x++) {
            POINT p1, p2;
            p1.row = esp_random() % SSD1306_HEIGHT;
            p1.col = esp_random() % SSD1306_WIDTH;
            p2.row = esp_random() % SSD1306_HEIGHT;
            p2.col = esp_random() % SSD1306_WIDTH;
            SSD1306_DrawLine(ssd1306, p1, p2);
            vTaskDelay(1);
        }

        vTaskDelay(50);
        SSD1306_ClearDisplay(ssd1306);
    }
}

