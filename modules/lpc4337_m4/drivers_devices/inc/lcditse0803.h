/*
 * FIUNER - 2018
 *
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
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

/* Initials     Name
 * ---------------------------
 * GN		Gainza Nicolas
 * MVLAN	Molina Van Leeuwen Ana Pabla
 * IR		Riveros Ignacio
 */

#ifndef LCD_ITSE0803_H
#define LCD_ITSE0803_H

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup LCD_ITSE0803 LCD ITSE0803
 ** @{
 * @file lcditse0803.h

/* @author
 * |   	Initials	|   			Name			 |
 * |:--------------:|:-------------------------------|
 * | 	GN 	 		| 	Gainza Nicolas			  	 |
 * | 	MVLAN 		| 	Molina Van Leeuwen Ana Pabla |
 * | 	IR		 	| 	Riveros Ignacio				 |
 */

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "bool.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** @brief Initialization function of EDU-CIAA LCD Modules
 *
 * @return TRUE if no error
 */
bool LcdItsE0803Init(void);

/** @brief Function to display value a LCD Module for EDU-CIAA
 *
 * @param[in] value to show 0..999
 *
 * @return false if an error occurs (out of scale), in other case returns true
 */
bool LcdItsE0803Write(uint16_t value);

/** @brief Function to read a number in LCD Module
 *
 * @return number in LCD Module
 */
uint16_t LcdItsE0803Read(void);

/** @brief Function to turn off LCD Module
 *
 */
void LcdItsE0803Off(void);

/** @brief DeInitialization function of EDU-CIAA LCD Module
 *
 * @return TRUE if no error
 */
bool LcdItsE0803DeInit(void);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef DISPLAYITS_E0803_H */

