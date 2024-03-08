/* Copyright 2024,
 * Esteban Osella
 * esteban.osella@uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
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

/*
 * Initials     Name
 * ---------------------------
 * EO Esteban Osella
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20240220 v0.0.1 EO initial version
 */

/*!\mainpage
*
* \section genDesc Descripción de la aplicacion
*	Sistema de generaci'on / simulaci'on y envio de datos de sEMG,
*	encoder, y presi'on sobre los pedales.
*	Estos datos son enviados por UART USB con cabecera propia
*	segun el tipo de dato.
*	Si bien el encoder tiene la posibilidad de ser utilizado a
*	una frecuencia superior, a fin de simplificar la implementaci'on
*	se hace un downsampling a la frecuencia de los pedales (80Hz).
*
* ---
*
* \section Sec01 Uso de HW:
*   \subsection subSec01 sEMG:
*   + if simulating: LPC_TIMER1
*   + with ADS1299: SPI port and GPIO_LCD_2,GPIO_LCD_3,GPIO_LCD_4,GPIO_LCD_EN,
	GPIO_LCD_RS
*   \subsection subSec02 Stimulator
*   + LPC_TIMER0
*   + GPIO_1 as stimulation trigger
*   + DAC0 as analog value for the stimulator.
*
*  ToDo:
*  + A method should be included to online change the ADC gain by serial port
*  or either GPIO
*
*/

/*==================[inclusions]=============================================*/
#include "uart.h"
#include "systick.h"
#include "delay.h"
#include "dsp_EMG.h"
#include <arm_math.h>
#include "led.h"
//#include "serial_headers.h"
#include "sEMG.h"
#include "stimulator.h"

#include "serial_headers.h"

/*==================[macros and definitions]=================================*/

// serial communication datarate
#define UART_BAUD_RATE  576000 // 921600// 576000


// SAMPLING PERIOD OF EACH VARIABLE
#define PEDAL_RATE 		80
#define ENCODER_RATE 	80

#define PEDAL_NCHAN		2
#define ENCODER_NCHAN	1

//resolution in bytes
#define PEDAL_RESOL		3
#define ENCODER_RESOL	2 // it is a 12 bits dac, but lets consider it as a 16 bit.
/*==================[internal data declaration]==============================*/

uint8_t stimulating = 0;
static uint16_t demand = 0;

/*==================[internal functions declaration]=========================*/
void SystickInt(void);


/*==================[internal data definition]===============================*/



/*===========================================================================*/

RINGBUFF_T rbRx;

#define PEDAL_BUFFER_SIZE	PEDAL_NCHAN*PEDAL_RESOL
#define ENCODER_BUFFER_SIZE	ENCODER_NCHAN*ENCODER_RESOL

// variables buffers
uint8_t PedalsBuff[PEDAL_BUFFER_SIZE+4] = {};
uint8_t EncoderBuff[ENCODER_BUFFER_SIZE+4] = {};

uint8_t pedalsDrdy = 0;
uint8_t EncoderDrdy = 0;


// static uint16_t pedals_ind = 0, encoder_ind = 0;
/*==================[external data definition]===============================*/
extern uint8_t sEMGDrdy;
/*==================[internal functions definition]==========================*/



void SysInit(void)
{
	SystemCoreClockUpdate();
	Chip_SetupIrcClocking();
	LedsInit();
	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};

	UartInit(&serial_init);
    SystickInit(500, SystickInt);
    fpuInit();//Enable FPU

}

void SystickInt(void)
{

}

void interruption_tec_1(void){
	// check wether the fw or the hw is ok, since it's not responding as expected
}

void interruption_tec_2(void){
	demand= demand+50;
	if (demand >1023)
		demand = 1023;
	StimUpdateDemand(0, demand);
}

void interruption_tec_3(void){
	demand= demand-50;
	if (demand >1023)
		demand = 0;

	StimUpdateDemand(0, demand);
}

void interruption_tec_4(void){
	if (stimulating)
	{
		stimulating = 0;
		StimDisable(0);
	}
	else
	{
		stimulating = 1;
		StimEnable(0);
	}
}


void DacInit2(){
	/* Initialize the DAC peripheral */
	Chip_DAC_Init(LPC_DAC);
	Chip_Clock_EnableOpts(CLK_APB3_DAC, true, true, 1);
	/* Set update rate to 400 KHz */
	Chip_DAC_SetBias(LPC_DAC, DAC_MAX_UPDATE_RATE_400kHz);

	/* Enables the DMA operation and controls DMA timer */
	Chip_DAC_ConfigDAConverterControl(LPC_DAC, DAC_DMA_ENA);
	/* DCAR DMA access */
	/* Update value to DAC buffer*/
	Chip_DAC_UpdateValue(LPC_DAC, 0);
}


/*==================[external functions definition]==========================*/
int main(void)
{

	SysInit();

	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};
	UartInit(&serial_init);
	//DacInit2();
	ConfigADS();

	uint16_t res = 0;
	uint8_t * val;//
	uint32_t outval = 0;
    // gpio input initialization

    GPIOInit(GPIO_TEC_1, GPIO_INPUT);
    GPIOActivInt( GPIOGP0 , GPIO_TEC_1, interruption_tec_1 , 0); // 0 <- IRQ_EDGE_FALL

    GPIOInit(GPIO_TEC_2, GPIO_INPUT);
    GPIOActivInt( GPIOGP0 , GPIO_TEC_2, interruption_tec_2 , 0); // 0 <- IRQ_EDGE_FALL

    GPIOInit(GPIO_TEC_3, GPIO_INPUT);
    GPIOActivInt( GPIOGP1 , GPIO_TEC_3, interruption_tec_3 , 0); // 0 <- IRQ_EDGE_FALL

    GPIOInit(GPIO_TEC_4, GPIO_INPUT);
    GPIOActivInt( GPIOGP2 , GPIO_TEC_4, interruption_tec_4 , 0); // 0 <- IRQ_EDGE_FALL
    StimInit();

	while(1)
	{
		if(pedalsDrdy)
		{
			for(res = 0;res  < 4 + PEDAL_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, PedalsBuff+res);
			pedalsDrdy = 0;
		}
		if(sEMGGetDRDY())
		{
			val = sEMGGetBuffer();
			outval = 0;
			for(res = 0;res  < 4 + sEMG_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, *(val+res));
			sEMGSetDRDY(0);
		}
		if(EncoderDrdy)
		{
			for(res = 0;res  < 4 + ENCODER_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, EncoderBuff+res);
			EncoderDrdy = 0;
		}
		if(StimGetTrxFlag())
		{
			val = StimGetBuffer();
			for(res = 0;res  < 4 + STIM_BUFF_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, *(val+res));
			StimSetTrxFlag(0);
		}
	}
	return 0;
}
/*==================[end of file]============================================*/

