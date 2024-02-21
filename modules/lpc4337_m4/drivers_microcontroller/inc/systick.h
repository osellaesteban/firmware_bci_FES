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

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Microcontroller Drivers microcontroller
 ** @{ */
/** @addtogroup systick
 *  @{
 *  @file systick.h
 *  @brief
 *  @section changeLog Historial de modificaciones (la mas reciente arriba)
 *
 *  20181008 v0.1 version inicial SM
 *
 *  Iniciales  |     Nombre
 * :----------:|:----------------
 *  SM		   | Sebastian Mateos
 *
 *  @date 08/10/2018
 */

#ifndef SYSTICK_H
#define SYSTICK_H



/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "bool.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/** @fn bool SystickInit(uint32_t ms, void * ptrIntFunc)
 * @brief Inicializacion del timer Systick
 * @param[in] ms Indica cada cuantos milisegundos se va a llamar a la funcion
 * @param[in] ptrIntFunc Funcion a la que se va a llamar periodicamente
 * @return Booleano que indica si se pudo inicializar el systick
 */
bool SystickInit(uint32_t ms, void *ptrIntFunc);
/** @fn bool SystickInit(uint32_t us, void * ptrIntFunc)
 * @brief Inicializacion del timer Systick
 * @param[in] us Indica cada cuantos microgundos se va a llamar a la funcion
 * @param[in] ptrIntFunc Funcion a la que se va a llamar periodicamente
 * @return Booleano que indica si se pudo inicializar el systick
bool SystickInitUs(uint32_t us, void *ptrIntFunc);
/** @fn void SystickDeinit (void)
 * @brief Deshabilita la interrupcion del systick
 */
void SystickDeinit(void);
/*==================[end of file]============================================*/
#endif /* INC_SYSTICK_H_ */

/** @}*/
/** @}*/
/** @}*/
