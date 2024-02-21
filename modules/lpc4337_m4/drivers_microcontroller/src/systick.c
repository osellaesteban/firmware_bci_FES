/* Copyright 2018,
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

#include "systick.h"
#include "chip.h"
#include <stdint.h>
#include "bool.h"

/** @addtogroup systick
 *  @{
 */

static bool init_systick = false; /**< Indica si esta inicializada la interrupcion del systick*/
static uint32_t miliSec = 0; /**< Variable utilizada por el contador para decrementar los milisegundos*/
static uint32_t uSec = 0; /**< Variable utilizada por el contador para decrementar los milisegundos*/

void (*ptrSystickFunc)(); /**< Puntero a la funcion que se va a llamar en la interrupcion del systick*/
void (*ptrContadorFunc)(); /**< Puntero a la funcion que se va a llamar en la funcion Contador*/

/** @fn void SystickCounter(void)
 * @brief Decrementa un contador en caso que los ms sean mayores que lo que puede manejar el systick
 */
void SystickCounter(void);

/** @fn void SysTick_Handler(void)
 * @brief Funcion de interrupcion del Systick, llama a una funcion periodicamente
 */
void SysTick_Handler(void);

bool SystickInit(uint32_t ms, void *ptrIntFunc)
{
	uint32_t ticks;
	ticks = ((SystemCoreClock/1000)*ms);
	if(ticks > 10000000)
	{
		ptrSystickFunc = SystickCounter;
		ptrContadorFunc = ptrIntFunc;
		miliSec = ms;
		SysTick_Config(SystemCoreClock / 1000);
		init_systick = true;
	}
	else if (ticks < 10)
		init_systick = false;
	else
	{
		ptrSystickFunc = ptrIntFunc;
		SysTick_Config(ticks);
		init_systick = true;
	}
	return init_systick;
}

bool SystickInitUs(uint32_t us, void *ptrIntFunc)
{
	uint32_t ticks;
	ticks = ((SystemCoreClock/1000000)*us);
	if(ticks > 10000000)
	{
		ptrSystickFunc = SystickCounterUs();
		ptrContadorFunc = ptrIntFunc;
		uSec = us;
		SysTick_Config(SystemCoreClock / 1000000);
		init_systick = true;
	}
	else if (ticks < 10)
		init_systick = false;
	else
	{
		ptrSystickFunc = ptrIntFunc;
		SysTick_Config(ticks);
		init_systick = true;
	}
	return init_systick;
}


void SysTick_Handler(void)
{
	ptrSystickFunc();
}

void SystickCounter(void)
{
	static uint32_t cont;
	if(init_systick)
	{
		cont = miliSec;
		init_systick = false;
	}
	if(cont > 0)
		cont--;
	else
	{
		ptrContadorFunc();
		cont = miliSec;
	}
}


void SystickCounterUs(void)
{
	static uint32_t cont;
	if(init_systick)
	{
		cont = uSec;
		init_systick = false;
	}
	if(cont > 0)
		cont--;
	else
	{
		ptrContadorFunc();
		cont = uSec;
	}
}


void SystickDeinit(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; /**< Desactiva Systick */
}

/** @}*/
