/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// HAL_I2C.h - Prototypes of hardware abstraction layer for I2C between
//             MSP432P401R and OPT3001
//
//****************************************************************************

#ifndef __HAL_I2C_H_
#define __HAL_I2C_H_

#include <driverlib.h>

#define I2C_BASE            EUSCI_B1_BASE

#define I2C_SCL_PORT        GPIO_PORT_P4
#define I2C_SCL_PIN         GPIO_PIN1

#define I2C_SDA_PORT        GPIO_PORT_P4
#define I2C_SDA_PIN         GPIO_PIN0

#define I2C_SELECT_FUNCTION GPIO_SECONDARY_MODULE_FUNCTION

void I2C_initGPIO(void);
void I2C_init(void);
bool I2C_write8(unsigned char pointer, unsigned char writeByte, unsigned int timeout);
bool I2C_write16(unsigned char pointer, unsigned short writeWord, unsigned int timeout);
bool I2C_read8(unsigned char pointer, char *result, unsigned int timeout);
bool I2C_read16(unsigned char pointer, short *result, unsigned int timeout);
void I2C_setslave(unsigned short slaveAdr);

#endif /* __HAL_I2C_H_ */
