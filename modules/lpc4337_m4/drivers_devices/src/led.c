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

/** \brief Bare Metal driver for leds in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 * EF		Eduardo Filomena
 * JMR		Juan Manuel Reta
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version Leando Medus
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20180210 v0.4 modifications and improvements made by Sebastian Mateos
 * 20190820 v1.1 new version made by Sebastian Mateos
 */

/*==================[inclusions]=============================================*/
#include "led.h"
#include "gpio.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

uint8_t LedsInit(void)
{
	/** Configuration of the GPIO */
	GPIOInit(GPIO_LED_1, GPIO_OUTPUT);
	GPIOInit(GPIO_LED_2, GPIO_OUTPUT);
	GPIOInit(GPIO_LED_3, GPIO_OUTPUT);
	GPIOInit(GPIO_LED_RGB_R, GPIO_OUTPUT);
	GPIOInit(GPIO_LED_RGB_G, GPIO_OUTPUT);
	GPIOInit(GPIO_LED_RGB_B, GPIO_OUTPUT);

	/** Turn off leds*/
	GPIOOff(GPIO_LED_1);
	GPIOOff(GPIO_LED_2);
	GPIOOff(GPIO_LED_3);
	GPIOOff(GPIO_LED_RGB_R);
	GPIOOff(GPIO_LED_RGB_G);
	GPIOOff(GPIO_LED_RGB_B);

	return true;
}

/** \brief Function to turn on a specific led */
uint8_t LedOn(uint8_t led)
{
	uint8_t result = false;
	switch (led)
	{
	case LED_1:
		GPIOOn(GPIO_LED_1);
		result = true;
		break;
	case LED_2:
		GPIOOn(GPIO_LED_2);
		result = true;
		break;
	case LED_3:
		GPIOOn(GPIO_LED_3);
		result = true;
		break;
	case LED_RGB_R:
		GPIOOn(GPIO_LED_RGB_R);
		result = true;
		break;
	case LED_RGB_G:
		GPIOOn(GPIO_LED_RGB_G);
		result = true;
		break;
	case LED_RGB_B:
		GPIOOn(GPIO_LED_RGB_B);
		result = true;
		break;
	}
	return result;
}

uint8_t LedOff(uint8_t led)
{
	uint8_t result = false;
	switch (led)
	{
	case LED_1:
		GPIOOff(GPIO_LED_1);
		result = true;
		break;
	case LED_2:
		GPIOOff(GPIO_LED_2);
		result = true;
		break;
	case LED_3:
		GPIOOff(GPIO_LED_3);
		result = true;
		break;
	case LED_RGB_R:
		GPIOOff(GPIO_LED_RGB_R);
		result = true;
		break;
	case LED_RGB_G:
		GPIOOff(GPIO_LED_RGB_G);
		result = true;
		break;
	case LED_RGB_B:
		GPIOOff(GPIO_LED_RGB_B);
		result = true;
		break;
	}
	return result;
}

uint8_t LedToggle(uint8_t led)
{
	uint8_t result = false;

	switch (led)
	{
	case LED_1:
		GPIOToggle(GPIO_LED_1);
		result = true;
		break;
	case LED_2:
		GPIOToggle(GPIO_LED_2);
		result = true;
		break;
	case LED_3:
		GPIOToggle(GPIO_LED_3);
		result = true;
		break;
	case LED_RGB_R:
		GPIOToggle(GPIO_LED_RGB_R);
		result = true;
		break;
	case LED_RGB_G:
		GPIOToggle(GPIO_LED_RGB_G);
		result = true;
		break;
	case LED_RGB_B:
		GPIOToggle(GPIO_LED_RGB_B);
		result = true;
		break;
	}

	return result;
}

uint8_t LedsOffAll(void)
{
	GPIOOff(GPIO_LED_1);
	GPIOOff(GPIO_LED_2);
	GPIOOff(GPIO_LED_3);
	GPIOOff(GPIO_LED_RGB_R);
	GPIOOff(GPIO_LED_RGB_G);
	GPIOOff(GPIO_LED_RGB_B);
	
	return true;
}

uint8_t LedsMask(uint8_t mask)
{
	GPIOState(GPIO_LED_1,(mask & LED_1));
	GPIOState(GPIO_LED_2,(mask & LED_2));
	GPIOState(GPIO_LED_3,(mask & LED_3));
	GPIOState(GPIO_LED_RGB_R,(mask & LED_RGB_R));
	GPIOState(GPIO_LED_RGB_G,(mask & LED_RGB_G));
	GPIOState(GPIO_LED_RGB_B,(mask & LED_RGB_B));

	return true;
}

/*==================[end of file]============================================*/
