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

/** \brief Bare Metal driver for GPIO in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20180226 v0.1 initials initial version SM
 */

/*==================[inclusions]=============================================*/

#include "i2c.h"
#include "chip.h"


/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

 uint8_t Init_I2c( uint32_t clockRateHz ){

      /* Configuracion de las lineas de SDA y SCL de la placa*/
      Chip_SCU_I2C0PinConfig( I2C0_STANDARD_FAST_MODE );
      /* Inicializacion del periferico*/
      Chip_I2C_Init( I2C0 );
      /* Seleccion de velocidad del bus*/
      Chip_I2C_SetClockRate( I2C0, clockRateHz );
      /* Configuracion para que los eventos se resuelvan por polliong
      // (la otra opcion es por interrupcion)*/
      Chip_I2C_SetMasterEventHandler( I2C0, Chip_I2C_EventHandlerPolling );

      return true;
   }

uint8_t I2CRead( 				  uint8_t  i2cSlaveAddress,
                                  uint8_t* dataToReadBuffer,
                                  uint16_t dataToReadBufferSize,
								  bool   sendWriteStop,
                                  uint8_t* receiveDataBuffer,
                                  uint16_t receiveDataBufferSize,
								  bool   sendReadStop ){

      I2CM_XFER_T i2cData;

      i2cData.slaveAddr = i2cSlaveAddress;
      i2cData.options   = 0;
      i2cData.status    = 0;
      i2cData.txBuff    = dataToReadBuffer;
      i2cData.txSz      = dataToReadBufferSize;
      i2cData.rxBuff    = receiveDataBuffer;
      i2cData.rxSz      = receiveDataBufferSize;

      if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ) {
         return FALSE;
      }

      return TRUE;
   }

uint8_t I2CWrite( 				   uint8_t  i2cSlaveAddress,
                                   uint8_t* transmitDataBuffer,
                                   uint16_t transmitDataBufferSize,
								   bool   sendWriteStop ){


      I2CM_XFER_T i2cData;

      // Prepare the i2cData register
      i2cData.slaveAddr = i2cSlaveAddress;
      i2cData.options   = 0;
      i2cData.status    = 0;
      i2cData.txBuff    = transmitDataBuffer;
      i2cData.txSz      = transmitDataBufferSize;
      i2cData.rxBuff    = 0;
      i2cData.rxSz      = 0;

      /* Send the i2c data */
      if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ){
         return FALSE;
      }

      return TRUE;
   }



/*==================[external functions definition]==========================*/

/*==================[ISR external functions definition]======================*/

/*==================[end of file]============================================*/
