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
#include "serial_headers.h"
#include "sEMG.h"
#include "controller.h"
#include "stimulator.h"


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


// serial receiving definitions
#define RX_BUFF_SIZE 8 // HEAD DESCRIPTOR DATA*5 TAIL


uint16_t AngleBias = 0;
uint16_t MeassuredAngle = 0;

/*===================[external data declaration]=============================*/

//extern STIM_BUFF_SIZE;
/*==================[internal data declaration]==============================*/

uint8_t stimulating = 0;

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

uint8_t RxBuff[RX_BUFF_SIZE];
uint8_t RxResend = 0;
// static uint16_t pedals_ind = 0, encoder_ind = 0;
/*==================[external data definition]===============================*/
extern uint8_t sEMGDrdy;
/*==================[internal functions definition]==========================*/


uint32_t FatigueStat = 0;
void interruption_tec_1(void){
	// check wether the fw or the hw is ok, since it's not responding as expected
}
void LimitsInitilization();
void interruption_tec_2(void){
	MeassuredAngle = MeassuredAngle+10;
	if (MeassuredAngle > 360)
		MeassuredAngle = 0;
	ControlUpdateAngle(MeassuredAngle);

}

void interruption_tec_3(void){
	MeassuredAngle = MeassuredAngle-10;
	if (MeassuredAngle > 360)
		MeassuredAngle = 360;
	ControlUpdateAngle(MeassuredAngle);
}

void interruption_tec_4(void){
	FatigueStat+=10;
	ControlSetMuscleFatigue(QUAD, FatigueStat);
}

void GPIOInitialization(){
	GPIOInit(GPIO_TEC_1, GPIO_INPUT);
	GPIOActivInt( GPIOGP0 , GPIO_TEC_1, interruption_tec_1 , 0); // 0 <- IRQ_EDGE_FALL

	GPIOInit(GPIO_TEC_2, GPIO_INPUT);
	GPIOActivInt( GPIOGP0 , GPIO_TEC_2, interruption_tec_2 , 0); // 0 <- IRQ_EDGE_FALL

	GPIOInit(GPIO_TEC_3, GPIO_INPUT);
	GPIOActivInt( GPIOGP1 , GPIO_TEC_3, interruption_tec_3 , 0); // 0 <- IRQ_EDGE_FALL

	GPIOInit(GPIO_TEC_4, GPIO_INPUT);
	GPIOActivInt( GPIOGP2 , GPIO_TEC_4, interruption_tec_4 , 0); // 0 <- IRQ_EDGE_FALL
}

void SysInit(void)
{
	SystemCoreClockUpdate();
	Chip_SetupIrcClocking();
	LedsInit();
	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};

	UartInit(&serial_init);
    SystickInit(500, SystickInt);
    fpuInit();//Enable FPU

    // gpio input initialization
    GPIOInitialization();
}

void SystickInt(void)
{
	ControlUpdateAngle(MeassuredAngle+AngleBias);
	ControlUpdateStimulation();
	ControlMotorUpdate();
}



void UpdateFatigueMsg(void){
	muscle_t muscle = (muscle_t ) RxBuff[2];
	FatigueStat = (((uint32_t) RxBuff[3]) << 24) + (((uint32_t) RxBuff[4]) << 16) +
			(((uint32_t) RxBuff[5]) << 8 ) + (((uint32_t) RxBuff[6]));
	ControlSetMuscleFatigue(muscle,FatigueStat);
}

void UpdateAngleBiasMsg(void){
	AngleBias = (((uint32_t) RxBuff[2]) << 8 ) + (((uint32_t) RxBuff[2]));
}

void UpdateControlMsg(){

}

void Resend()
{
	static uint8_t CommandBuff[4];
	CommandBuff[0] = HEAD;
	CommandBuff[1] = COMMAND_HEAD;
	CommandBuff[2] = COMMAND_RESEND;
	CommandBuff[3] = TAIL;
	UartSendBuffer(SERIAL_PORT_PC, CommandBuff,4) ;
	RxResend = 0;
/*
	for(res = 0;res  < 4; res++)
		UartSendByte(SERIAL_PORT_PC, controlBuff+res);*/
}



/*==================[external functions definition]==========================*/
int main(void)
{

	SysInit();

	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};
	UartInit(&serial_init);
	ConfigADS();

	uint16_t res = 0;
	uint8_t * val;//
	uint8_t rval=0;

    // StimInit();
	ControlInitialize();

    uint8_t buff_ptr = 0;
	while(1)
	{
		if(pedalsDrdy)
		{
			UartSendBuffer(SERIAL_PORT_PC, PedalsBuff, 4 + PEDAL_BUFFER_SIZE) ;
			/*for(res = 0;res  < 4 + PEDAL_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, PedalsBuff+res);*/
			pedalsDrdy = 0;
		}
		if(sEMGGetDRDY())
		{
			val = sEMGGetBuffer();
			UartSendBuffer(SERIAL_PORT_PC, val, 4 + sEMG_BUFFER_SIZE) ;
			/* for(res = 0;res  < 4 + sEMG_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, *(val+res));*/
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
			UartSendBuffer(SERIAL_PORT_PC, val,4 + STIM_BUFF_SIZE) ;
			/*for(res = 0;res  < 4 + STIM_BUFF_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, *(val+res));*/
			StimSetTrxFlag(0);
		}
		if(ControlGetTrxFlag())
		{
			val = ControlGetBuffer();
			UartSendBuffer(SERIAL_PORT_PC, val,4 + CTRL_BUFF_SIZE) ;
			StimSetTrxFlag(0);
		}
		if (UartRxReady(SERIAL_PORT_PC))
		{
			UartReadByte(SERIAL_PORT_PC,&rval);
			if (rval == HEAD)
			{
				RxBuff[buff_ptr] = HEAD;
				for (buff_ptr = 1; buff_ptr <RX_BUFF_SIZE ; buff_ptr++)
				{
					UartReadByte(SERIAL_PORT_PC,&rval);
					RxBuff[buff_ptr] = rval;
				}

				buff_ptr = 0;
				switch (RxBuff[1]){
				case COMMAND_HEAD:
					UpdateControlMsg();
					break;
				case FATIGUE_HEAD:
					UpdateFatigueMsg();
					break;
				case COMMAND_ANGLE_BIAS:
					UpdateAngleBiasMsg();
					break;
				default:
					RxResend = 1;
					Resend();
					break;
				}
			}
		}
	}
	return 0;
}





/*==================[end of file]============================================*/

