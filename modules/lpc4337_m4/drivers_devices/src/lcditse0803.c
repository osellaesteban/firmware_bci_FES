/*
 * FIUNER - 2018
 *
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
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



/*
 * Initials     Name
 * ---------------------------
 * GN		Gainza Nicolas
 * MVLAP	Molina Van Leeuwen Ana Pabla
 * IR		Riveros Ignacio
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20180510 v0.1 GN, MVLAP, IR initial version
 * 20160807 v0.2 SM modifications and improvements
 */


/** \brief Bare Metal driver for LCD Poncho in the EDU-CIAA board.
 **
 **/

/*==================[inclusions]=============================================*/
#include "lcditse0803.h"
#include "gpio.h"
#include "chip.h"


/*==================[macros and definitions]=================================*/

static uint16_t actual_value = 0; /*variable that saves the value to be shown in the display LCD*/

bool LcdItsE0803Init(void)
{
	/* Configuration of pins of data*/
	GPIOInit(GPIO_LCD_1, GPIO_OUTPUT);
	GPIOInit(GPIO_LCD_2, GPIO_OUTPUT);
	GPIOInit(GPIO_LCD_3, GPIO_OUTPUT);
	GPIOInit(GPIO_LCD_4, GPIO_OUTPUT);

	/* Configuration of pins of control*/
	GPIOInit(GPIO_1, GPIO_OUTPUT);
	GPIOInit(GPIO_3, GPIO_OUTPUT);
	GPIOInit(GPIO_5, GPIO_OUTPUT);

	actual_value=0;
	LcdItsE0803Write(actual_value);
	return true;
};


/** @brief Aux function to load a digit to the LCD Display
 *
 */
bool LcdItsE0803BCDtoPin(uint8_t value)
{
	GPIOState(GPIO_LCD_1, (value & (1<<0))>>0);
	GPIOState(GPIO_LCD_2, (value & (1<<1))>>1);
	GPIOState(GPIO_LCD_3, (value & (1<<2))>>2);
	GPIOState(GPIO_LCD_4, (value & (1<<3))>>3);
	return true;
}

 bool LcdItsE0803Write(uint16_t value)
 {
	 uint8_t units, tens, hundreds;

	 if(value<1000)
	 {
		 actual_value = value;

		 hundreds = value/100;
		 tens = (value-(hundreds*100))/10;
		 units = (value-(hundreds*100)-(tens*10));

		 /* Write hundreds */
		 LcdItsE0803BCDtoPin(hundreds);
		 GPIOOn(GPIO_1);
		 GPIOOff(GPIO_1);

		 /* Write tens */
		 LcdItsE0803BCDtoPin(tens);
		 GPIOOn(GPIO_3);
		 GPIOOff(GPIO_3);

		 /* Write units */
		 LcdItsE0803BCDtoPin(units);
		 GPIOOn(GPIO_5);
		 GPIOOff(GPIO_5);


		 return true; /* return 1 for values lower than 999 */
	 }
	 else
		 return false; /* return 0 for values higher than 999 */
 	 }

uint16_t LcdItsE0803Read(void)
{
	return (actual_value);
}

void LcdItsE0803Off(void)
{
	LcdItsE0803BCDtoPin(0x0F);
	 GPIOOn(GPIO_1);
	 GPIOOff(GPIO_1);

	LcdItsE0803BCDtoPin(0x0F);
	 GPIOOn(GPIO_3);
	 GPIOOff(GPIO_3);

	 LcdItsE0803BCDtoPin(0x0F);
	 GPIOOn(GPIO_5);
	 GPIOOff(GPIO_5);

}

bool LcdItsE0803DeInit(void)
{
	GPIODeinit();
	return true;
}

/*==================[end of file]============================================*/
