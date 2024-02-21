/* Copyright 2019,
 * Electrónica Programable
 * jmrera@ingenieria.uner.edu.ar
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


#ifndef ANALOG_IO_H
#define ANALOG_IO_H
/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup Analog_IO Analog IO
 ** @{ */

/** \brief AD Converter Bare Metal driver for the peripheral in the EDU-CIAA Board.
 **
 ** This is a driver to control the peripheral Analogs Inputs/Outputs
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *	JM			Juan Manuel Reta
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190517 v0.1 initials initial version JM
 */

/*==================[inclusions]=============================================*/
#include "stdint.h"


/*==================[macros]=================================================*/
#define CH0 1  /*Board connector: DAC pin*/
#define CH1 2  /*Board connector: CH1 pin*/
#define CH2 4  /*Board connector: CH2 pin*/
#define CH3 8  /*Board connector: CH3 pin*/

#define DAC	0    /*Board connector: DAC pin. Override CH0 declaration*/

#define AINPUTS_SINGLE_READ	0
#define AINPUTS_BURST_READ	1

#define ANALOG_INPUT_READ	1
#define ANALOG_OUTPUT_WROTE  1

#define ERROR_CHANNEL_SELECTION 0
#define ANALOG_OUTPUT_RANGE 1023
/*==================[typedef]================================================*/

typedef struct {				/*!< Analog Inputs config struct*/
		uint8_t input;			/*!< inputs: CH0, CH1, CH2, CH3*/
		uint8_t mode;			/*!< mode: Single Read, burst read.*/
		void *pAnalogInput;	/*!< Function pointer to app analog_inputs convert event*/
} analog_input_config;


/*==================[external data declaration]==============================*/


/*==================[external functions declaration]=========================*/
/** @fn void AnalogInputInit(analog_input_config *inputs)
 * @brief Single analog input initialization
 * @param[in] inputs Initilization parameters struct: channel
 * @return null
 */
void AnalogInputInit(analog_input_config *inputs);

/** @fn void AnalogOutputInit(void)
 * @brief Analog output initialization - DAC
 * @return null
 */
void AnalogOutputInit(void);

/** @fn void AnalogInputReadPooling(uint8_t channel, uint16_t *value)
 * @brief Read single channel by polling.
 * @param[in] channel Channel selected.
 * @param[in] value Read variable pointer
 * @return null
 */
void AnalogInputReadPolling(uint8_t channel, uint16_t *value);

/** @fn void AnalogStartConvertion(void)
 * @brief Start convertion for ADC0 module
 * @return null
 */
void AnalogStartConvertion(void);


/** @fn void AnalogInputRead(uint8_t channel, uint16_t value)
 * @brief Funtion to use IRQ, read single channel
 * @param[in] channel Channel selected.
 * @param[in] value Read variable pointer
 * @return null
 */
void AnalogInputRead(uint8_t channel, uint16_t *value);


/** @fn void AnalogOutputWrite(uint16_t value)
 * @brief Digital-to-Analog convert - DAC pin.
  * @param[in] value Digital value to convert
 * @return null
 */
uint8_t AnalogOutputWrite(uint16_t value);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef ANALOG_IO_H */

