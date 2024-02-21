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

/** \brief Bare Metal driver for hc_sr04 for the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190226 v0.1 SM initial version
 */

/*==================[inclusions]=============================================*/
#include "hc_sr04.h"
#include "chip.h"
#include "delay.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static gpio_t echo_st, trigger_st; /**<  Stores the pin inicilization*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/

bool HcSr04Init(gpio_t echo, gpio_t trigger)
{
	echo_st = echo;
	trigger_st = trigger;

	/** Configuration of the GPIO pins*/
	GPIOInit(echo, GPIO_INPUT);
	GPIOInit(trigger, GPIO_OUTPUT);

	return true;
}

uint16_t HcSr04ReadDistanceInCentimeters(void)
{
	uint16_t distance = 0;

	GPIOOn(trigger_st);
	DelayUs(10);
	GPIOOff(trigger_st);
	while(!GPIORead(echo_st));
	do
	{
		DelayUs(1);
		distance++;
		if(distance > 23200)
			return 400;
	}
	while(GPIORead(echo_st));
	return (distance/32);
}

uint16_t HcSr04ReadDistanceInInches(void)
{
	uint16_t distance = 0;

	GPIOOn(trigger_st);
	DelayUs(10);
	GPIOOff(trigger_st);
	while(!GPIORead(echo_st));
	do
	{
		DelayUs(1);
		distance++;
		if(distance > 23200)
			return 157;
	}
	while(GPIORead(echo_st));
	return (distance/148);
}

bool HcSr04Deinit(void)
{
	GPIODeinit();
	return true;
}

/*==================[end of file]============================================*/
