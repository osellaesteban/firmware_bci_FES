/* Copyright 2016, 
 * Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 * Sebastián Mateos
 * sebastianantoniomateos@gmail.com
 *
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


#ifndef UART_H
#define UART_H

/** \brief UART Bare Metal driver for the peripheral in the EDU-CIAA Board.
 **
 ** This is a driver to control the UART present in the EDU-CIAA Board.
 **
 **/

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Microcontroller Drivers microcontroller
 ** @{ */
/** \addtogroup UART UART
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160610 v0.1 initials initial version leo
 */

/*==================[inclusions]=============================================*/
#include "stdint.h"
#include "chip.h"
//#include "srting.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

#define SERIAL_PORT_PC				0
#define SERIAL_PORT_P2_CONNECTOR	1
#define SERIAL_PORT_RS485			2

#define NO_INT	0		/*NULL pointer for no interrupt port handler*/

typedef struct {				/*!< Serial Ports Struct*/
		uint8_t port;			/*!< port: FTDI, RS232, RS485*/
		uint32_t baud_rate;		/*!< Baud Rate*/
		void *pSerial;			/*!< Function pointer to app serial port*/
} serial_config;


/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** @fn uint32_t UartInit(serial_config *port)
 * @brief Inicializar UART
 * @param[in] port Puerto para inicializar (FTDI, RS232, RS485)
 */
uint32_t UartInit(serial_config *port);

/** @fn uint32_t UartReadStatus(uint8_t port)
 * @brief Leer el estado del puerto
 * @param[in] port Puerto que se desea leer
 */
uint32_t UartReadStatus(uint8_t port);
uint32_t UartRxReady(uint8_t port);
uint8_t UartReadByte(uint8_t port, uint8_t* dat);
void UartSendByte(uint8_t port,uint8_t* dat);
void UartSendString(uint8_t port, uint8_t *msg);/*Stops on '\0'*/
void UartSendBuffer(uint8_t port, const void *data, uint8_t nbytes);  /*Send n Bytes - Inline candidate...*/

/** @fn char* UartItoa(uint32_t val, uint8_t base)
 * @brief Conversor de entero a ASCII
 * @param[in] val Valor entero que se desea convertir
 * @param[in] base Base sobre la cual se desea realizar la conversion
 * @return Puntero al primer elemento de la cadena convertida
 */
uint8_t* UartItoa(uint32_t val, uint8_t base);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef UART_H */

