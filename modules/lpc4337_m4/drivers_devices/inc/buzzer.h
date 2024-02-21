/* Copyright 2019,
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
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
#ifndef BUZZER_H
#define BUZZER_H

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup Buzzer
 ** @{ */

/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/
#define lpc4337            1
#define mk60fx512vlq15     2

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/


/*==================[external functions declaration]=========================*/

/** @brief Initialization function
 ** 
 ** @return TRUE if no error
 **/
bool BuzzerInit(void);


/** @brief Function to turn on buzzer
 **/
void BuzzerOn(void);

/** @brief Function to turn off buzzer
 **/
void BuzzerOff(void);

/** @brief Function to change buzzer tone
 ** param[in] freq Frequency of tone of the buzzer
 **/
void BuzzerSetFrec(uint16_t freq);

/** @brief Deinitialization function
 **
 ** @return TRUE if no error
 **/
bool BuzzerDeinit(void);
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/
#endif /* #ifndef BUZZER_H */

