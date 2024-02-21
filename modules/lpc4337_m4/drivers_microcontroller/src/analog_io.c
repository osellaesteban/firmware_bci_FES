/* Copyright 2019,
 * Electrónica Programable
 * jmrera@ingenieria.uner.edu.ar
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

/** \brief Bare Metal driver for adc in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *	JM			Juan Manuel Reta
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190517 v0.1 initials initial version JM
 */

/*==================[inclusions]=============================================*/
#include "analog_io.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/
#define DAC_RESOLUTION	10
#define DAC_MAX_VALUE 	(1<<DAC_RESOLUTION)-1
/*==================[internal data declaration]==============================*/
void (*pIsrAnalogInput)();
/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
 void ADC0_IRQHandler()
 {
	 pIsrAnalogInput();
}

/*==================[external functions definition]==========================*/
/** \brief Analog Input Initialization  */
void AnalogInputInit(analog_input_config *inputs)
{
	static ADC_CLOCK_SETUP_T configADC;

	configADC.adcRate=1000;		/** max 409 KHz*/
	configADC.burstMode=DISABLE;
	configADC.bitsAccuracy=ADC_10BITS;

	Chip_ADC_Init(LPC_ADC0,&configADC);
	Chip_ADC_SetSampleRate(LPC_ADC0, &configADC,ADC_MAX_SAMPLE_RATE);

	if(!(inputs->input ^ CH0))
	{
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH0,ENABLE);
		if(inputs->pAnalogInput != NULL)
		{
			pIsrAnalogInput=inputs->pAnalogInput;
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,ADC_CH0,ENABLE);
			NVIC_EnableIRQ(ADC0_IRQn);
		}
	}
	if(!(inputs->input ^ CH1))
	{
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH1,ENABLE);
		if(inputs->pAnalogInput != NULL)
		{
			pIsrAnalogInput=inputs->pAnalogInput;
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,ADC_CH1,ENABLE);
			NVIC_EnableIRQ(ADC0_IRQn);
		}
	}
	if(!(inputs->input ^ CH2))
	{
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH2,ENABLE);
		if(inputs->pAnalogInput != NULL)
		{
			pIsrAnalogInput=inputs->pAnalogInput;
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,ADC_CH2,ENABLE);
			NVIC_EnableIRQ(ADC0_IRQn);
		}
	}
	if(!(inputs->input ^ CH3))
	{
		Chip_ADC_EnableChannel(LPC_ADC0,ADC_CH3,ENABLE);
		if(inputs->pAnalogInput != NULL)
		{
			pIsrAnalogInput=inputs->pAnalogInput;
			Chip_ADC_Int_SetChannelCmd(LPC_ADC0,ADC_CH3,ENABLE);
			NVIC_EnableIRQ(ADC0_IRQn);
		}
	}
}

void AnalogOutputInit(void)
{
	/** DAC function selection */
	Chip_SCU_DAC_Analog_Config();

	/** DAC initialization */
	Chip_DAC_Init(LPC_DAC);
	Chip_DAC_SetBias(LPC_DAC, DAC_MAX_UPDATE_RATE_400kHz);
	Chip_DAC_SetDMATimeOut(LPC_DAC, 0xffff);
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_CNT_ENA | DAC_DMA_ENA);
}

void AnalogInputReadPolling(uint8_t channel, uint16_t *value)
{
	/** Start Acquisition */
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	switch(channel)
	{
	case CH0:
		while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH0, ADC_DR_DONE_STAT) != SET);
		/** Conversion complete, and value reading */
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH0, value);
		break;
	case CH1:
		while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH1, ADC_DR_DONE_STAT) != SET);
		/** Conversion complete, and value reading */
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH1, value);
		break;
	case CH2:
		while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH2, ADC_DR_DONE_STAT) != SET);
		/** Conversion complete, and value reading */
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH2, value);
		break;
	case CH3:
		while (Chip_ADC_ReadStatus(LPC_ADC0, ADC_CH3, ADC_DR_DONE_STAT) != SET);
		/** Conversion complete, and value reading */
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH3, value);
		break;
	}
}

void AnalogStartConvertion(void)
{
  Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
}

void AnalogInputRead(uint8_t channel, uint16_t *value)
{
	switch(channel)
	{
	case CH0:
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH0, value);
		break;
	case CH1:
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH1, value);
		break;
	case CH2:
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH2, value);
		break;
	case CH3:
		Chip_ADC_ReadValue(LPC_ADC0,ADC_CH3, value);
		break;
	}
}


uint8_t AnalogOutputWrite(uint16_t value)
{
	if( value >= DAC_MAX_VALUE)
	{
		value = DAC_MAX_VALUE;
		return ANALOG_OUTPUT_RANGE;
	}
	Chip_DAC_UpdateValue(LPC_DAC,(uint32_t) value);
	return ANALOG_OUTPUT_WROTE;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
