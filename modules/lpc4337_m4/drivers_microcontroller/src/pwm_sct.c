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

/** \brief Bare Metal driver for buzzer in the EDU-CIAA board.
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
 * 20190520 v0.1 SM initial version
 */

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "pwm_sct.h"

/*==================[macros and definitions]=================================*/
/** @typedef digitalIO
 * @brief Config options for pwm
 */
typedef struct
{
	uint8_t hwPort; /**< Port of hardware*/
	uint8_t hwPin; /**< Pin of hardware*/
	uint16_t mode; /**< Pin mode*/
} digitalIO;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
const digitalIO pwm[] =
{
	{0x04, 0x02, FUNC1}, /**< Configuration CTOUT0*/
	{0x04, 0x01, FUNC1}, /**< Configuration CTOUT1*/
	{0x04, 0x04, FUNC1}, /**< Configuration CTOUT2*/
	{0x04, 0x03, FUNC1}  /**< Configuration CTOUT3*/
};
bool ctout_active[sizeof(pwm)]; /**< Saves the CTOUTs used*/
uint8_t duty_cycle_saved[sizeof(pwm)]; /**< Saves the CTOUTs duty cycle*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/


/*==================[external functions definition]==========================*/
bool PWMInit(pwm_out_t * ctout, uint8_t n_outs, uint16_t freq)
{
	uint8_t i;
	Chip_SCTPWM_Init(LPC_SCT);
	Chip_SCTPWM_SetRate(LPC_SCT, freq);
	for(i=0 ; i<n_outs ; i++)
	{
		ctout_active[ctout[i]] = true;
		Chip_SCU_PinMux(pwm[ctout[i]].hwPort , pwm[ctout[i]].hwPin , SCU_MODE_INACT , pwm[ctout[i]].mode);
		Chip_SCTPWM_SetOutPin(LPC_SCT, ctout[i]+1 , ctout[i]);
		Chip_SCTPWM_SetDutyCycle(LPC_SCT, ctout[i]+1, Chip_SCTPWM_PercentageToTicks(LPC_SCT, 0));
	}
	return true;
}

void PWMOn(void)
{
	Chip_SCTPWM_Start(LPC_SCT);
}

/** @brief Function to turn off PWM outputs
 **/
void PWMOff(void)
{
	Chip_SCTPWM_Stop(LPC_SCT);
}

void PWMSetDutyCycle(pwm_out_t ctout, uint8_t duty_cycle)
{
	duty_cycle_saved[ctout] = duty_cycle;
	Chip_SCTPWM_SetDutyCycle(LPC_SCT, ctout+1, Chip_SCTPWM_PercentageToTicks(LPC_SCT, duty_cycle));
}

bool PWMSetFreq(uint32_t freq)
{
	uint8_t i;
	Chip_SCTPWM_SetRate(LPC_SCT, freq);
	Chip_SCTPWM_Start(LPC_SCT);
	for(i=0 ; i<sizeof(ctout_active) ; i++)
		if(ctout_active[i])
			Chip_SCTPWM_SetDutyCycle(LPC_SCT, i+1, Chip_SCTPWM_PercentageToTicks(LPC_SCT, duty_cycle_saved[i]));
	return true;
}

bool PWMDeinit(void)
{
	Chip_SCTPWM_Stop(LPC_SCT);
	return true;
}
/*==================[end of file]============================================*/
