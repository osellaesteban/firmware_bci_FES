/* Copyright 2016, 
 * Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
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


#ifndef SWITCH_H
#define SWITCH_H


/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup Switch
 ** @{ */

/** @brief Bare Metal header for switches in EDU-CIAA NXP
 **
 ** This is a driver for four switches mounted on the board
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 *  EF			Eduardo Filomena
 *  JMR			Juan Manuel Reta
 *  SM			Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena 
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20180210 v0.4 modifications and improvements made by Sebastian Mateos
 * 20190820 v1.1 new version made by Sebastian Mateos
 */

/*==================[inclusions]=============================================*/
#include "bool.h"
#include <stdint.h>

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/
enum SWITCHES {SWITCH_1=(1<<0), SWITCH_2=(1<<1), SWITCH_3=(1<<2), SWITCH_4=(1<<3)};

/*==================[external functions declaration]=========================*/
/** @brief Initialization function to control basic push-buttons in the EDU-CIAA BOARD
 * This function initialize the four switches present in the EDU-CIAA board
 * @return true if no error
 */
int8_t SwitchesInit(void);

/** @brief Function to read basic push-buttons
 * The function returns a 8 bits word, where the first four binaries positions represent each push-button.
 * @return 0 if no key pressed, SWITCH_1 SWITCH_2 SWITCH_3 SWITCH_4 in other case.
 */
int8_t SwitchesRead(void);

/** @brief Enables the interruption of a particular key and assigns a group of interrupts defined in @ref switchgp_t
 * @param[in] gp Indicates to which group of interrupts the key is assigned
 * @param[in] tec Key that will interrupt
 * @param[in] ptrIntFunc Function to be called in the interruption
 */
void SwitchActivInt(uint8_t tec, void *ptrIntFunc);

/** @brief Activate group interrupts and add to the interruption the chosen keys through the mask
 * @param[in] tecs Mask of keys that will be added to the group to interrupt. Where bit0-TEC4, bit1-TEC3, bit2-TEC2 and bit4-TEC1.
 * @param[in] void * Function to be called in the interruption.
 */
void SwitchesActivGroupInt(uint8_t tecs, void *ptrIntFunc);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef SWITCH_H */

