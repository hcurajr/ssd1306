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
#ifndef __result_codes_h__
#define __result_codes_h__

typedef enum _RESULT {
    COORDINATE_OUT_OF_RANGE,
    FAILED_READ,
    FAILED_TO_ALLOCATE_MEMORY,
    FAILED_TO_CONFIGURE_I2C_PINS,
    FAILED_TO_INSTALL_ISR_FUNCTION,
    FAILED_TO_INSTALL_ISR_SERVICE,
    FAILED_TO_SET_INTERRUPT_TYPE,
    FAILED_TO_SET_PIN_LEVEL,
    FAILED_TO_SEND_SLAVE_ADDRESS,
    FAILED_WRITE_RECEIVED_NACK,
    INVALID_ARGUMENT,
    INVALID_CONTEXT,
    INVALID_SCL_PIN,
    INVALID_SDA_PIN,
    INVALID_SLAVE_ADDRESS_GT7F,
    INVALID_SLAVE_ADDRESS_RESERVED,
    INVALID_STATE_CHANGE_REQUEST,
    NOT_IMPLEMENTED,
    OK
}RESULT;

#endif //__result_codes_h__
