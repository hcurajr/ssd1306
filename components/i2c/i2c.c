/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
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
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "freertos/FreeRTOS.h"
#include "esp8266/eagle_soc.h"
#include "esp8266/gpio_struct.h"
#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "result_codes.h"
#include "i2c.h"
#include "timer_util.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Following macro used to validate and assign context to ptr
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
#define ASSIGN_CONTEXT(ptr, context, func_name)                             \
if((context)==NULL) {                                                       \
    ESP_LOGE(I2C_TAG, "%s: context pointer cannot be NULL.", func_name);    \
    return INVALID_ARGUMENT;                                                \
}                                                                           \
if(((i2c_type_t *)(context))->opaque != (uint32_t)(context)) {              \
    ESP_LOGE(I2C_TAG, "%s: context pointer corrupt. %u != %u",              \
                func_name, (uint32_t)context,                               \
                ((i2c_type_t *)(context))->opaque);                         \
    return INVALID_ARGUMENT;                                                \
}                                                                           \
ptr = (i2c_type_t *)context
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  GPIO is defined in gpio_struct.h
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define GPIO_GET_LEVEL(x)       (GPIO.in >> (x)) & 0x1
#define GPIO_SET_LEVEL_LOW(x)   GPIO.out_w1tc |= (0x1 << (x))
#define GPIO_SET_LEVEL_HIGH(x)  GPIO.out_w1ts |= (0x1 << (x))
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * see setState method for state transition logic.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef enum _I2C_STATE { ACK, READY, START, STOP, WRITE } I2C_STATE;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Private context pointer structure used to manage I2C communication. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef struct _i2c_type_t {   
    uint32_t                opaque;         // address of structure - safety check prior to free.
    uint8_t                 slaveAddress;   // only supports 7-bit addresses.
    gpio_num_t              scl;            // SCL pin - control line.
    gpio_num_t              sda;            // SDA pin - data line. 
    bool                    useAck;         // if false, 9th SCL cycle not used for ACK result.
    volatile I2C_STATE      state;          // state of I2C transmission. 
    volatile uint32_t       isrAckCount;    // captures how many times ISR processed an ACK result.
                                            // Only set by ISR. Read-only for tasks.
}i2c_type_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Following typedef used to pickout correct version of byte-writer
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
typedef RESULT (*WRITEFN)(i2c_type_t *, const uint8_t);
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

static const char *I2C_TAG = "I2C";

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Forward reference private methods.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */ 
static RESULT enableACK(i2c_type_t *ptr);
static RESULT writeByte(i2c_type_t *ptr, const uint8_t byte);
static RESULT setState(i2c_type_t *ptr, const I2C_STATE newState);
static const char *stateToString(I2C_STATE state);
#if 0 
static uint32_t calculateRiseTime(const gpio_num_t pin);
static uint32_t calculateFallTime(const gpio_num_t pin);
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * The following ISR is used to capture ACK result from slave. ACKs occur when 
 * slave pulls SDA line LOW. This ISR is configured to trigger on a falling edge.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static void 
ReadAckISR(void *context)
{
    i2c_type_t *ptr = (i2c_type_t *)context;
    uint32_t status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);   // see gpio_register.h
    GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status);       // clear interrupt flag
    if (ptr->state == ACK) {
        ptr->state = WRITE;
        ptr->isrAckCount++;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Public method used to initialize I2C structure. 
 *
 *  INPUT
 *      slaveAddress - device to communicate with. Address must not be left-shifted. 
 *               scl - SCL pin number.
 *               sda - SDA pin number.
 *            useAck - if true will call enableACK() to initialize ISR and set write method.
 *           context -  pointer to pointer to store context structure.
 *
 *  OUTPUT
 *      OK  success or failure code.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT 
I2C_Initialize(const uint8_t slaveAddress, const gpio_num_t scl, const gpio_num_t sda, const bool useAck, void **context) 
{
    #ifdef DEBUG
    ESP_LOGD(I2C_TAG, "I2C_Initialize(): Inputs slaveAddress: 0x%x, SCL:%d, SDA:%d.", slaveAddress, scl, sda);
    #endif

    if (context == NULL) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): context pointer cannot be null.");
        return INVALID_CONTEXT;    
    }

    if (!GPIO_IS_VALID_GPIO(scl)) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): SCL Pin %d not valid GPIO PIN.", scl);
        return INVALID_SCL_PIN;   
    }

    if (!GPIO_IS_VALID_GPIO(sda)) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): SDA Pin %d not valid GPIO PIN.", sda);
        return INVALID_SDA_PIN;   
    }

    uint8_t shiftedAddress = slaveAddress << 1;

    // Check for any of following reserved addresses
    //  0x00        general call address
    //  0x01        start byte
    //  0x02-0x03   CBUS address mask
    //  0x04-0x05   reserved for different bus format mask
    //  0x06-0x07   reserved for future purpose mask
    //  0x08-0x0F   Hs-mode master code  
    //  0xF0-0xF7   10-bit slave addressing  
    //  0xF9-0xFF   device ID  
    if ((shiftedAddress <= 0x0F) || 
        (shiftedAddress >= 0xF0 && shiftedAddress <= 0xF7) || 
        (shiftedAddress == 0xF9 || shiftedAddress == 0xFB || shiftedAddress == 0xFD || shiftedAddress == 0xFF)) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): Shifted slaveAddress (0x%x) cannot be a reserved address.", shiftedAddress);
        return INVALID_SLAVE_ADDRESS_RESERVED;
    }

    // only support seven bit addressing scheme.
    if (shiftedAddress > 0x7F) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): Shifted slaveAddress (0x%x) cannot be greater than 0x7F.", shiftedAddress);
        return INVALID_SLAVE_ADDRESS_GT7F;
    }

    i2c_type_t *ptr = (i2c_type_t *)malloc(sizeof(i2c_type_t));
    if (ptr == NULL) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): Failed to allocate memory for context!");
        return FAILED_TO_ALLOCATE_MEMORY;    
    }

    gpio_config_t io_conf = {0};
    io_conf.pin_bit_mask = (1ULL << scl) | (1ULL << sda);
    io_conf.mode         = GPIO_MODE_OUTPUT; 
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE; 
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    esp_err_t esp_result = gpio_config(&io_conf);
    if (esp_result != ESP_OK) {
        ESP_LOGE(I2C_TAG, "I2C_Initialize(): Failed to configure pins SDA:%d, SCL:%d. ESP Error: %d", sda, scl, esp_result);
        return FAILED_TO_CONFIGURE_I2C_PINS;
    }
    
    // Setting pins to OUTPUT causes the pin levels to go LOW.
    // Reset them both HIGH sda followed by scl so that we don't 
    // trigger device logic.
    GPIO_SET_LEVEL_HIGH(sda);
    GPIO_SET_LEVEL_HIGH(scl);

    #ifdef DEBUG
    ESP_LOGD(I2C_TAG, "I2C_Initialize(): SCL:%d and SDA:%d pins configured successfully.", scl, sda);
    #endif

    ptr->opaque       = (uint32_t)ptr;      // self-reference the pointer address
    ptr->slaveAddress = slaveAddress << 1;  // make room for r/w bit in lowest most significant digit
    ptr->scl          = scl;
    ptr->sda          = sda;
    ptr->state        = READY;
    ptr->isrAckCount  = 0;

    if (useAck == true) {
        RESULT result = enableACK(ptr);
        if (result != OK) {
            I2C_FreeContext((void **)&ptr);
            return result;
        }
    } 

    *context = (void *)ptr;
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Public method called to safely free context pointer. 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT I2C_FreeContext(void **context)
{
    if (context == NULL) {
        return OK;    
    }

    i2c_type_t *ptr;
    ASSIGN_CONTEXT(ptr, *context, "I2C_FreeContext");

    if (ptr->opaque != (uint32_t)ptr) {
        ESP_LOGE(I2C_TAG, "FreeContext(): Invalid i2c pointer provided, will not free memory!");
        return INVALID_CONTEXT;
    }

    free(ptr);
    *context = NULL;
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Public method to send START condition to slave and validate slaveAddress.
 *
 *  NOTE
 *      Must be called prior to any transmission. 
 *      Input state: READY. 
 *      Output state: START.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
I2C_StartXmit(void *context) 
{
    i2c_type_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "I2C_StartXmit");

    RESULT result = setState(ptr, START);
    if (result != OK) {
        ESP_LOGE(I2C_TAG, "I2C_StartXmit(): Failed to change state to START. Error = %d.", result);
        return result;
    }
 
    GPIO_SET_LEVEL_LOW(ptr->sda);      
    delay(TICKS_IN_600_NS);                         // tHD;STA                         
    GPIO_SET_LEVEL_LOW(ptr->scl);

    uint8_t address = ptr->slaveAddress & 0xFE;     // turn off lsb to signal write op
    result = writeByte(ptr, address);
    if (result != OK) {
        ESP_LOGE(I2C_TAG, "I2C_StartXmit(): Failed to write address byte (0x%x) to slave. Error=%d", address, result);
    }

    return result;   
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Public method to send STOP condition to slave.
 *
 *  NOTE
 *      Must be called after all transmission is complete.
 *      Input state: WRITE. 
 *      Output state: READY.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
I2C_StopXmit(void *context) 
{
    i2c_type_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "I2C_Stop");

    RESULT result = setState(ptr, READY);           
    if (result != OK) {
        ESP_LOGE(I2C_TAG, "I2C_StopXmit(): failed to change state to READY. Error = %d.", result);
        return result;
    }

    GPIO_SET_LEVEL_HIGH(ptr->scl);       
    delay(TICKS_IN_600_NS);                         // tSU;STO
    GPIO_SET_LEVEL_HIGH(ptr->sda);       
    delay(TICKS_IN_1300_NS);                        // tBUF
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Public method to write multiple bytes to slave device.
 *  
 *  NOTE
 *      Ability to send multiple bytes may require device-specific support.
 *      For example, ssd1306 display device requires a control byte to 
 *      precede every data/command byte sent to the device. This must be 
 *      taken into account by caller when calling this method.
 *
 *  Following are timing definitions for different parts of I2C communication.    
 *  Times are all in nano-seconds. The comments identify the canonical labels   
 *  from the I2C spec. 
 *
 *  NOTE
 *      bit-banging algo will account for min times but will not attempt to 
 *      address for jitter on SCL line. 
 *      
 *      Based on testing, these times don't appear to matter for 
 *      software-based implementation. 
 *                                                     
 *  ----------------------------------------------------------------------------------------------- 
 *  VALUE   TIME@100kHZ TIME@400kHZ MIN/MAX CONDITION   DESCRIPTION
 *  -----------------------------------------------------------------------------------------------
 *  tHD;STA    4000ns      600ns      MIN     START     SDA holds LOW for this amount of time
 *                                                      before SCL channel goes LOW i.e. sends
 *                                                      out the first clock pulse.   
 *  tSU;DAT     250ns      100ns      MIN     DATA-XMIT Data setup time. Amount of time for
 *                                                      SDA to remain stable before SCL channel
 *                                                      switches to HIGH.
 *  tHD;DAT       0ns        0ns      MIN     DATA-XMIT Data hold time. Represents the amount
 *                                                      of time to wait after SCL goes low but
 *                                                      prior to SDA channel changing.
 *  tVD;DAT    3450ns      900ns      MAX     DATA-XMIT Data valid time. Defines the amount of
 *                                                      time for a data transition to take place
 *                                                      as measured from time SCL is LOW. 
 *  tSU;STO    4000ns      600ns      MIN     STOP      Stop Condition setup time. Amount of 
 *                                                      time SCL will remain HIGH before SCL 
 *                                                      switches to HIGH.
 *  tVD;ACK    3450ns      900ns      MAX     ACK       ACK data valid time. Similar to tVD;DAT.
 *  tBUF       4700ns     1300ns      MIN               Idle time between a STOP & START condition.
 *  tHIGH      4000ns      600ns      MIN               Length of HIGH period of the SCL clock.
 *  tLOW       4700ns     1300ns      MIN               Length of LOW period of the SCL clock.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
RESULT
I2C_Write(void *context, const uint8_t byte)
{
    i2c_type_t *ptr;
    ASSIGN_CONTEXT(ptr, context, "I2C_Write");
  
    RESULT result = setState(ptr, WRITE);
    if (result != OK) {
        ESP_LOGE(I2C_TAG, "I2C_Write(): Failed to change state to WRITE. Error = %d.", result);
        return result;
    }

    result = writeByte(ptr, byte); 
    if (result != OK) {
        ESP_LOGE(I2C_TAG, "I2C_Write(): Failed to write byte 0x%x. Error=%d", byte, result);
    }
  
    setState(ptr, STOP);        // signal that bytes were sent and STOP is possible. 
    return result;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 * Private method used to Write a single byte to wire and captures ACK result from slave.
 * Must keep interface consistent with writeByteNoAck() method.
 *
 *  INPUTS
 *      ptr         Points to i2c context
 *      byteToSend  Contents of 8 bits to send
 *
 *  OUTPUT
 *      OK          Success. SCL and SDA will exit in LOW state.
 *                  If sda exits function in HIGH state, this means ISR 
 *                  received NACK from slave device.
 *
 *  NOTES
 *      scl and sda must come in LOW
 *      testing with SSD1306 showed the following:
 *          1. Delays for tSU;DAT and tHIGH are not necessary.
 *          2. Delay for tVD;ACK is required to capture ACK result.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT
writeByte(i2c_type_t *ptr, const uint8_t byteToSend) 
{
    uint16_t bitMask = 0x0100; 
    uint8_t bitToSend;

    while (bitMask != 0x1) {
        bitMask >>= 1;
        bitToSend = byteToSend & bitMask;

        if (bitToSend>0) {
            GPIO_SET_LEVEL_HIGH(ptr->sda);      
        } else {
            GPIO_SET_LEVEL_LOW(ptr->sda);    
        }

        GPIO_SET_LEVEL_HIGH(ptr->scl);          // allow data capture by slave
        GPIO_SET_LEVEL_LOW(ptr->scl);           // finish capture
    }

    // NOTE: Slaves will grab the sda line immediately after the 8th bit is set.
    //       If the 8 bits represent an address then there will be an imperceptible
    //       delay in checking address pertains to device for it to grab the sda
    //       line. Handle ACK by going HIGH on SDA, but for a slave ACKing byte, 
    //       the line will not actually go HIGH, and this will be visible in logic
    //       analyzer or oscilloscope.
    setState(ptr, ACK);                         // Allow ISR to capture ACK result 
    GPIO_SET_LEVEL_HIGH(ptr->sda);              // Release sda line. For ACK, line will most likely already be LOW
    GPIO_SET_LEVEL_HIGH(ptr->scl);              // ACK is not sent until SCL line high 
    GPIO_SET_LEVEL_LOW(ptr->scl);               // finish capture, slave will now release sda line 
    GPIO_SET_LEVEL_LOW(ptr->sda);
    delay(TICKS_IN_100_NS);                     // a small delay is required to give time for ISR to process ACK

    if (ptr->state == ACK) {
        setState(ptr, WRITE);                   // ERROR: return error condition but leave it up to caller whether 
                                                // to continue sending bytes.  
        ESP_LOGE(I2C_TAG, "writeByte(): Received NACK. Failed to write 0x%x to slave. isrAckCount=%d.", 
                            byteToSend, ptr->isrAckCount);
        return FAILED_WRITE_RECEIVED_NACK;
    }

    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Private method called by initialize to enable ACK functionality.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT
enableACK(i2c_type_t *ptr) 
{
    // configure ISR service to use per-gpio handlers
    esp_err_t result = gpio_install_isr_service(0);
    switch (result) {
        case ESP_OK:                
        case ESP_ERR_INVALID_STATE: // gpio.h states ESP_ERR_INVALID_STATE is return value for 'ISR service already installed' 
        case ESP_FAIL:              // error. gpio.c however, shows that ESP_FAIL will be returned for this condition.
            break;                  // Regardless treat both as success.
        case ESP_ERR_NOT_FOUND:     // gpio.h states this value will be returned if there's no free interrupt found with
                                    // specified flags. However, gpio.c does not show this value being returned at all
                                    // nor does it check for this case.
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR Service. Reason unknown. Error=%d.", result);
            return FAILED_TO_INSTALL_ISR_SERVICE;
        case ESP_ERR_INVALID_ARG:   // gpio.c will return this value only if the global gpio_intr_service function is null.
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR Service because global service function is null. Error=%d.", result);
            return FAILED_TO_INSTALL_ISR_SERVICE;
        case ESP_ERR_NO_MEM:        // no memory to allocate ISR service.
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR Service due to lack of memory. Error=%d.", result);
                return FAILED_TO_INSTALL_ISR_SERVICE;
    };

    // Configure the gpio pin to trigger when SDA line is LOW which signals ACK response.
    result = gpio_set_intr_type(ptr->sda, GPIO_INTR_NEGEDGE);
    if (result != ESP_OK) { 
        ESP_LOGE(I2C_TAG, "enableACK(): Failed to set interrupt type. Invalid SDA pin (%d) likely culprit. Error = %d.", ptr->sda, result);
        return FAILED_TO_SET_INTERRUPT_TYPE;
    }
 
    // install the ISR
    result = gpio_isr_handler_add(ptr->sda, ReadAckISR, (void *)ptr);
    switch (result) {
        case ESP_OK:
            break;
        // neither of these two conditions is possible because we've already
        // validated the pins and the ISR service were successfully installed
        // prior to calling the isr-add function. Handling for completeness.
        case ESP_ERR_INVALID_STATE: 
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR, must install ISR Service first. Error=%d.", result); 
            return FAILED_TO_INSTALL_ISR_FUNCTION;
        case ESP_ERR_INVALID_ARG:
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR, Invalid SDA pin (%d) likely culprit. Error=%d.", ptr->sda, result);
            return FAILED_TO_INSTALL_ISR_FUNCTION;
        default:
            ESP_LOGE(I2C_TAG, "enableACK(): Failed to install ISR, encountered unknown error. Error=%d.", result);
            return FAILED_TO_INSTALL_ISR_FUNCTION; 
    }

    ptr->useAck = true;
    return OK;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Private method to change the I2C state. 
 *  
 *  Valid state transitions are:
 *
 *      READY-> START   state transition to intiatiate WRITE operation.
 *      START-> ACK     occurs durint START phase when slave address sent to device
 *      START-> WRITE   state transition to initiate WRITE operation. 
 *      WRITE-> ACK     this allows master to wait on ACK from slave.
 *        ACK-> WRITE   state transition initiated from ISR to unblock MASTER.
 *      WRITE-> STOP    completed WRITE operation.
 *       STOP-> READY   state transition to complete WRITE operation.
 *       STOP-> WRITE   can resume writing from STOP.
 * 
 *  This method will check the current state against requeted state and allow
 *  state transition thru if it's one of the valid transitions or fail request.
 *  
 *  NOTE
 *      ISR does not call this method because it pre-empts any task.
 *      ISR will move state from ACK->WRITE.
 *
 *  INPUT
 *      state:      requested new state
 *
 *  OUTPUT
 *      RESULT:    will equal OK if state transition successful or 
 *                 INVALID_STATE_CHANGE_REQUEST otherwise.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static RESULT
setState(i2c_type_t *ptr, const I2C_STATE newState) 
{
    if (newState == ptr->state) {
        return OK;
    }

    switch (ptr->state) {
        case ACK:
            switch (newState) {
                case WRITE:     
                    ptr->state = newState;
                    return OK;
                default:
                    break;        
            }
            break;

        case READY:
            switch (newState) {
                case START:
                    ptr->state = newState;
                    return OK;
                default:
                    break;        
            } 
            break;

        case START:
            switch (newState) {
                case ACK:       
                case WRITE:
                    ptr->state = newState;
                    return OK;
                default:
                    break;        
            } 
            break;

        case STOP:
            switch (newState) {
                case READY:
                case WRITE:
                    ptr->state = newState;
                    return OK;
                default:
                    break;
            } 

        case WRITE:
            switch (newState) {
                case ACK:
                case STOP:
                    ptr->state = newState;
                    return OK;
                default:
                    break;        
            }
            break;

        default:
            break;        
    }

    ESP_LOGE(I2C_TAG, "setState(): Invalid state change requested from '%s' to '%s'", 
                      stateToString(ptr->state), stateToString(newState));

    return INVALID_STATE_CHANGE_REQUEST; 
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Private method to return string representation of STATE.
 *  Note: Must be kept in sync with enum class I2C_STATE!
 *
 *  INPUT
 *      state:  state to prinout
 *
 *  OUTPUT
 *      pointer to constant char array with state as string.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static const char *
stateToString(I2C_STATE state) 
{
    switch (state) {
        case ACK:
            return "ACK";
        case READY:
            return "READY";
        case START:
            return "START";
        case STOP:
            return "STOP";
        case WRITE:
            return "WRITE";
        default:
            break;
    }

    return "ERROR!";
}

#if 0 
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Following two static methods are used to calculate the rise time of a pin going 
 *  from LOW to HIGH and vice-versa.
 *
 *  NOTE
 *      Have seen number vary first time call is made. Using small loop and
 *      taking last valid value which seems to be consistent after second call.
 *
 *  INPUT
 *      state: pin num  
 *
 *  OUTPUT
 *      rise time in ns calculated as follows:
 *          c1=CCOUNT prior to pin going high
 *          c2=CCOUNT after pin going to high
 *          time estimated as (c2-c1) * 12.5ns
 *          12.5ns = 1/80MHz 
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
uint32_t
calculateRiseTime(const gpio_num_t pin) 
{
    int32_t riseTime, c1, c2;
    for (int x=0; x<3; x++) {
        GPIO_SET_LEVEL_LOW(pin);
        c1 = getCCOUNT();
        GPIO_SET_LEVEL_HIGH(pin);
        c2 = getCCOUNT();
        riseTime = (uint32_t)(c2-c1);
    }
    #ifdef DEBUG
    ESP_LOGD(I2C_TAG, "calculatePortRiseTime(): Port Rise Time = %u cpu ticks. c1=%u, c2=%u", riseTime, c1, c2);
    #endif
    
    return riseTime;
}

uint32_t
calculateFallTime(const gpio_num_t pin) 
{
    int32_t fallTime, c1, c2;
    for (int x=0; x<3; x++) {
        GPIO_SET_LEVEL_HIGH(pin);
        c1 = getCCOUNT();
        GPIO_SET_LEVEL_LOW(pin);
        c2 = getCCOUNT();
        fallTime = (uint32_t)(c2-c1);
    }
    #ifdef DEBUG
    ESP_LOGD(I2C_TAG, "calculatePortFallTime(): Port Fall Time = %u cpu ticks. c1=%u, c2=%u", fallTime, c1, c2);
    #endif
    
    return fallTime;
}
#endif

