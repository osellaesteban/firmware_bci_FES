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
#ifndef PWM_SCT_H
#define PWM_SCT_H

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Microcontroller Drivers microcontroller
 ** @{ */
/** \addtogroup PWM_sct PWM sct
 ** @{ */


/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
typedef enum
{
	CTOUT0 = 0, /**< Located in TFIL_2*/
	CTOUT1,		/**< Located in TFIL_1*/
	CTOUT2,		/**< Located in LCD1*/
	CTOUT3		/**< Located in TFIL_3*/
}pwm_out_t;
/*==================[external data declaration]==============================*/


/*==================[external functions declaration]=========================*/

/** @brief Initialization function
 ** param[in] ctout vector with the outputs pin
 ** param[in] n_outs number of outputs
 ** param[in] freq Frequency of ALL PWM outputs
 ** @return TRUE if no error
 **/
bool PWMInit(pwm_out_t * ctout, uint8_t n_outs, uint16_t freq);

/** @brief Function to turn on PWM outputs
 **/
void PWMOn(void);

/** @brief Function to turn off PWM outputs
 **/
void PWMOff(void);

/** @brief Function to change PWM duty cycle of an output
 ** param[in] ctout Output pin
 ** param[in] duty_cycle Duty cycle of PWM
 **/
void PWMSetDutyCycle(pwm_out_t ctout, uint8_t duty_cycle);

/** @brief Function to set freq of all PWM outputs
 ** param[in] freq Frequency of PWM
 ** @return TRUE if no error
 **/
bool PWMSetFreq(uint32_t freq);

/** @brief Function to turn off PWM
 ** @return TRUE if no error
 **/
bool PWMDeinit(void);

/*==================[end of file]============================================*/
#endif /* #ifndef PWM_SCT_H */

