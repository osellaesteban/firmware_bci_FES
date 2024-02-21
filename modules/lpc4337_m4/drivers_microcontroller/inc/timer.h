/* Copyright 2016, XXXXXXXXXX
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

#ifndef TIMER_H
#define TIMER_H
/** \brief Bare Metal example header file
 **
 ** This is a mini example of the CIAA Firmware
 **
 **/

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Microcontroller Drivers microcontroller
 ** @{ */
/** \addtogroup Timer Timer
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "stdint.h"


/*==================[macros]=================================================*/
#define lpc4337            1
#define mk60fx512vlq15     2

#define TIMER_A 			0 				/*SysTick timer (CORTEX-M)*/
#define TIMER_B 			1 				/*RTI Timer (NXP)*/
#define TIMER_C 			2 				/*SysTick timer (CORTEX-M)*/

#define TIMER_A_1ms_TICK 	1000
#define TIMER_B_1ms_TICK 	1
#define TIMER_C_1us_TICK 	1000000


/*==================[typedef]================================================*/
typedef struct {				/*!< Timer Struct*/
		uint8_t timer;			/*!< Timer Seleccionado*/
		uint16_t period;		/*!< Periodo */
		void *pFunc;			/*!< Puntero a funcion que se llamara repetitivamente*/
} timer_config;
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** @brief Función de inicialización del Cronometro.
 * Configura el timer para trabajar con el SysTick (TIMER_A o TIMER_C) o con el RIT (TIMER_B).
 * No podran utilizarse el TIMER_A y el TIMER_C en simultaneo ya que ambos utlizan el mismo harware, la diferencia es que el TIMER_A
 * recibe el periodo en milisegundos y el TIMER_C recibe el periodo en microsegundos.
 *
 * @param[in] *timer_ini Puntero a estructura de inicializacion del timer.
 */
void TimerInit(timer_config *timer_ini);

/** @brief Funcion de comienzo de timer.
 * Se debe llamar siempre despues de TimerInit y cada vez que se haga un TimerStop.
 *
 * @param[in] timer Timer seleccionado.
 */
void TimerStart(uint8_t timer);

/** @brief Funcion de prada de timer.
 * Se debe llamar luego de TimerStart, para detener la temporizacion.
 *
 * @param[in] timer Timer seleccionado.
 */
void TimerStop(uint8_t timer);

/** @brief Funcion de reseteo del timer.
 * Se puede llamar en cualquier momento, reestablece a cero la cuenta del timer.
 *
 * @param[in] timer Timer seleccionado.
 */
void TimerReset(uint8_t timer);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef MI_NUEVO_PROYECTO_H */

/** @}*/
