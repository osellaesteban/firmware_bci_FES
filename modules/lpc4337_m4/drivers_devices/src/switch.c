/* Copyright 2019,
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

/** @brief Bare Metal driver for switchs in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *  SM			Sebastian Mateos
 *  EF			Eduardo Filomena
 *  JMR			Juan Manuel Reta
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
#include "switch.h"
#include "gpio.h"

/*==================[macros and definitions]=================================*/
#define LOW_EDGE 0
#define N_SW 4

/*==================[internal data declaration]==============================*/
void (*ptr_tec_int_func[4])(); /**< Pointer to the function to be called at the interruption of each key*/
void (*ptr_tec_group_int_func)(); /**< Pointer to the function to be called in the group interruption of the keys*/

/*==================[internal functions declaration]=========================*/

void Switch1Int (void);
void Switch2Int (void);
void Switch3Int (void);
void Switch4Int (void);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
int8_t SwitchesInit(void)
{
	/* GPIO configurations */
	GPIOInit(GPIO_TEC_1, GPIO_INPUT);
	GPIOInit(GPIO_TEC_2, GPIO_INPUT);
	GPIOInit(GPIO_TEC_3, GPIO_INPUT);
	GPIOInit(GPIO_TEC_4, GPIO_INPUT);

	return true;
}

int8_t SwitchesRead(void)
{
	int8_t mask = 0;
	if (!GPIORead(GPIO_TEC_1))
		  mask |= SWITCH_1;
	if (!GPIORead(GPIO_TEC_2))
		  mask |= SWITCH_2;
	if (!GPIORead(GPIO_TEC_3))
		  mask |= SWITCH_3;
	if (!GPIORead(GPIO_TEC_4))
		  mask |= SWITCH_4;
	return mask;
}

void SwitchActivInt(uint8_t sw, void *ptr_int_func)
{
	switch(sw)
	{
	case SWITCH_1:
		ptr_tec_int_func[0] = ptr_int_func;
		GPIOActivInt(GPIOGP0, GPIO_TEC_1, Switch1Int, LOW_EDGE);
		break;
	case SWITCH_2:
		ptr_tec_int_func[1] = ptr_int_func;
		GPIOActivInt(GPIOGP1, GPIO_TEC_2, Switch2Int, LOW_EDGE);
		break;
	case SWITCH_3:
		ptr_tec_int_func[2] = ptr_int_func;
		GPIOActivInt(GPIOGP2, GPIO_TEC_3, Switch3Int, LOW_EDGE);
		break;
	case SWITCH_4:
		ptr_tec_int_func[3] = ptr_int_func;
		GPIOActivInt(GPIOGP3, GPIO_TEC_4, Switch4Int, LOW_EDGE);
		break;
	}
}

void SwitchesActivGroupInt(uint8_t switchs, void *ptr_int_func)
{
	uint8_t n_act_sw = 0;
	gpio_t sw_int[N_SW];
	ptr_tec_group_int_func = ptr_int_func;
	if (switchs & SWITCH_1)
	{
		sw_int[n_act_sw] = GPIO_TEC_1;
		n_act_sw++;
	}
	if (switchs & SWITCH_2)
	{
		sw_int[n_act_sw] = GPIO_TEC_2;
		n_act_sw++;
	}
	if (switchs & SWITCH_3)
	{
		sw_int[n_act_sw] = GPIO_TEC_3;
		n_act_sw++;
	}
	if (switchs & SWITCH_4)
	{
		sw_int[n_act_sw] = GPIO_TEC_4;
		n_act_sw++;
	}
	GPIOActivGroupInt(GPIOGROUPGP0, sw_int, n_act_sw, ptr_int_func, LOW_EDGE);
}

void Switch1Int (void){
	ptr_tec_int_func[0]();
}

void Switch2Int (void){
	ptr_tec_int_func[1]();
}

void Switch3Int (void){
	ptr_tec_int_func[2]();
}

void Switch4Int (void){
	ptr_tec_int_func[3]();
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
