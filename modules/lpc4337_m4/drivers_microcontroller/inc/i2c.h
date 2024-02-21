/* Copyright 2018,
 * Sebastian Mateos
 * sebastianantoniomateos@gmail.com
 * Cátedra Electrónica Programable
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef _I2C_H_
#define _I2C_H_

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup I2c
 ** @{ */

/*==================[inclusions]=============================================*/

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"


/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** \brief Initialization function of EDU-CIAA I2c
 **
 ** \
 **
 ** \param[in] clockRateHz
 **
 ** \return TRUE if no error
 **/
uint8_t Init_I2c( uint32_t clockRateHz );

/** \brief Recive data function of EDU-CIAA I2c
 **
 ** \
 **
 ** \param[in] i2cSlaveAddress, dataToReadBuffer, dataToReadBufferSize, sendWriteStop, receiveDataBuffer, receiveDataBufferSize, sendReadStop
 **
 ** \return TRUE if no error
 **/
uint8_t I2CRead( uint8_t  i2cSlaveAddress,
                uint8_t* dataToReadBuffer,
                uint16_t dataToReadBufferSize,
				bool   sendWriteStop,
                uint8_t* receiveDataBuffer,
                uint16_t receiveDataBufferSize,
				bool   sendReadStop );

/** \brief Transmit data function of EDU-CIAA I2c
 **
 ** \
 **
 ** \param[in] i2cSlaveAddress, transmitDataBuffer, transmitDataBufferSize, sendWriteStop
 **
 ** \return TRUE if no error
 **/
uint8_t I2CWrite( uint8_t  i2cSlaveAddress,
                 uint8_t* transmitDataBuffer,
                 uint16_t transmitDataBufferSize,
				 bool   sendWriteStop );

/*==================[ISR external functions declaration]=====================*/

/*==================[end of file]============================================*/
#endif

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
