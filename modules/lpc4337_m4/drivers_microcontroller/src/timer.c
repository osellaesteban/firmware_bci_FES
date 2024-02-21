/* Copyright 2016, XXXXXXXXX  
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

/** \brief Blinking Bare Metal driver led
 **
 **
 **
 **/

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

#include "chip.h"
#include "timer.h"


/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/
uint32_t tick_timer_a = 0;
uint32_t tick_timer_b = 0;

timer_config *timer_a, *timer_b;

void (*pIsrTimerA)();
void (*pIsrTimerB)();
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
void TimerInit(timer_config *timer_ini)
{
	switch(timer_ini->timer)
	{
	 	case TIMER_A:
	 		SystemCoreClockUpdate();
	 		SysTick_Config(SystemCoreClock/TIMER_A_1ms_TICK);
	 		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; /*Stop Timer*/
	 		timer_a = timer_ini;
	 		pIsrTimerA=timer_ini->pFunc;
	 	break;

	 	case TIMER_B:
	 		Chip_RIT_Init(LPC_RITIMER);
	 		Chip_RIT_SetTimerInterval(LPC_RITIMER,TIMER_B_1ms_TICK);
	 		timer_b = timer_ini;
	 		pIsrTimerB=timer_ini->pFunc;
	 	break;

	 	case TIMER_C:
	 		SystemCoreClockUpdate();
	 		SysTick_Config(SystemCoreClock/TIMER_C_1us_TICK);
	 		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; /*Stop Timer*/
	 		timer_a = timer_ini;
	 		pIsrTimerA=timer_ini->pFunc;
	 	break;
	}
}

void SysTick_Handler(void)
{
	uint16_t period = timer_a->period;
	tick_timer_a ++;
	if ( tick_timer_a == period)
	{
		TimerReset(TIMER_A);
		pIsrTimerA();
	}
}

void RIT_IRQHandler(void)
{
	uint16_t period = timer_b->period; /*calc period in cents*/
	tick_timer_b ++;
	if ( tick_timer_b == period)
	{
		TimerReset(TIMER_B);
		pIsrTimerB();   /* Call app function */
	}
	Chip_RIT_ClearInt(LPC_RITIMER);
}

void TimerStart(uint8_t timer)
{

	switch(timer)
	{
	 	case TIMER_A:
	 		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk  | SysTick_CTRL_ENABLE_Msk;
	 	break;
	 	case TIMER_B:
	 		NVIC_EnableIRQ(RITIMER_IRQn);
	 	break;
	 	case TIMER_C:
	 		SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk  | SysTick_CTRL_ENABLE_Msk;
	 	break;
	}
}

void TimerStop(uint8_t timer)
{
	switch(timer)
	{
	 	case TIMER_A:
	 		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	 	break;
	 	case TIMER_B:
	 		NVIC_DisableIRQ(RITIMER_IRQn);
	 	break;
	 	case TIMER_C:
	 		SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
	 	break;
	}
}

void TimerReset(uint8_t timer)
{
	switch(timer)
	{
	 	case TIMER_A:
	 		tick_timer_a = 0;
	 	break;
	 	case TIMER_B:
	 		tick_timer_b = 0;
	 	break;
	 	case TIMER_C:
	 		tick_timer_a = 0;
	 	break;
	}
}




/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */

/*==================[end of file]============================================*/

