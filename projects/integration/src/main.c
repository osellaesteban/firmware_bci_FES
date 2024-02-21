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
*	Sistema de simulaci'on y envio de datos de sEMG, encoder, y
*	presi'on sobre los pedales.
*	Estos datos son enviados por UART USB con cabecera propia
*	segun el tipo de dato.
*	Si bien el encoder tiene la posibilidad de ser utilizado a
*	una frecuencia superior,
*	a fin de simplificar la implementaci'on se hace un
*	downsampling a la frecuencia del sEMG (2kHz).
*
* ---
*
* \subsection subSec01 Funcionamiento
*	Parpadeo del led azul RGB a una frecuencia de 1Hz. Abre
*	los canales UART ante cada nuevo dato, en las
*	interrupciones donde se simula la lectura.
*	Para utilizar en la EDU CIAA NXP, al ser un poncho para
*	dicha placa consultar la bibligrafía en la página
*	https://ponchodebiopotenciales.wordpress.com/ para más información
*
*/

/*==================[inclusions]=============================================*/
#include "ADS1299.h"
#include "led.h"
#include "uart.h"
#include "systick.h"
#include "delay.h"
#include "dsp_EMG.h"
#include <arm_math.h>

#include "serial_headers.h"
/*==================[macros and definitions]=================================*/

// serial communication datarate
#define UART_BAUD_RATE  576000 // 921600// 576000


// SAMPLING PERIOD OF EACH VARIABLE
#define PEDAL_RATE 		80
#define ENCODER_RATE 	7800
#define SEMG_RATE 		2000

#define PEDAL_NCHAN		2
#define SEMG_NCHAN 		4
#define ENCODER_NCHAN	1

//resolution in bytes
#define PEDAL_RESOL		3
#define sEMG_RESOL		3
#define ENCODER_RESOL	2 // it is a 12 bits dac, but lets consider it as a 16 bit.
/*==================[internal data declaration]==============================*/



/*==================[internal functions declaration]=========================*/
void SystickInt(void);
void ReadData(void);
void ConfigADS(void);

/*==================[internal data definition]===============================*/
bool new_data = false;
int32_t channel_data[8];
uint8_t state[3];

RINGBUFF_T rbRx;

#define PEDAL_BUFFER_SIZE	PEDAL_NCHAN*PEDAL_RESOL
#define sEMG_BUFFER_SIZE	SEMG_NCHAN*sEMG_RESOL
#define ENCODER_BUFFER_SIZE	ENCODER_NCHAN*ENCODER_RESOL

// variables buffers
uint8_t PedalsBuff[PEDAL_BUFFER_SIZE+4] = {};
uint8_t sEMGBuff[sEMG_BUFFER_SIZE+4] = {};
uint8_t EncoderBuff[ENCODER_BUFFER_SIZE+4] = {};

uint8_t pedalsDrdy = 0;
uint8_t sEMGDrdy = 0;
uint8_t EncoderDrdy = 0;


static uint16_t pedals_ind = 0,semg_ind= 0,encoder_ind = 0;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
/*/fn void SysInit(void)
 * \brief Inicializacion principal
 *
 */

void Configure_Timer0(){
	Chip_TIMER_Reset(LPC_TIMER0);

	// 2. Set Prescale value in PR
	Chip_TIMER_PrescaleSet(LPC_TIMER0,Chip_Clock_GetRate(CLK_MX_TIMER0) / 1000000 - 1);

	// 3. Set the count value in Match register(MR)
	// Match 0 -> pedal force registry
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, 0);
	Chip_TIMER_ResetOnMatchDisable(LPC_TIMER0, 0);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER0, 0);
	//uint32_t pedalMatch = (uint32_t) 1000000/( PEDAL_RATE); //Chip_TIMER_ReadPrescale(CLK_MX_TIMER1) / PEDAL_RATE);
	uint32_t pedalMatch = (uint32_t) Chip_Clock_GetRate(CLK_MX_TIMER0)/(LPC_TIMER0->PR * PEDAL_RATE);
	Chip_TIMER_SetMatch(LPC_TIMER0, 0, pedalMatch );

	// 4. Set appropriate value in MCR.
	Chip_TIMER_MatchEnableInt(LPC_TIMER0,0);
//	Chip_TIMER_MatchEnableInt(LPC_TIMER0,1);

	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0,0);
	Chip_TIMER_Reset(LPC_TIMER0);


	// 5. Enable the timer by configuring TCR.
	Chip_TIMER_Enable(LPC_TIMER0);

	// 6. Enable timer interrupt if needed.
	NVIC_EnableIRQ(TIMER0_IRQn);

}

void Configure_Timer1(){
	Chip_TIMER_Reset(LPC_TIMER1);

	// 2. Set Prescale value in PR
	Chip_TIMER_PrescaleSet(LPC_TIMER1,Chip_Clock_GetRate(CLK_MX_TIMER1) / 1000000 - 1);

	// 3. Set the count value in Match register(MR)
	// Match0 -> sEMG and Encoder registry
	Chip_TIMER_MatchEnableInt(LPC_TIMER1, 0);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1, 0);
	Chip_TIMER_StopOnMatchDisable(LPC_TIMER1, 0);
	uint32_t semgMatch = (uint32_t) (1000000/( SEMG_RATE)); //Chip_TIMER_ReadPrescale(CLK_MX_TIMER1)
	Chip_TIMER_SetMatch(LPC_TIMER1, 0, semgMatch);

	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER1,0);
	Chip_TIMER_Reset(LPC_TIMER1);

	// 5. Enable the timer by configuring TCR.
	Chip_TIMER_Enable(LPC_TIMER1);

	// 6. Enable timer interrupt if needed.
	NVIC_EnableIRQ(TIMER1_IRQn);

}


void TIMER0_IRQHandler(void)
{
	static uint32_t val, lost = 0,n_encoders= 0,n_pedals = 0;
	uint8_t UART_data,it = 0;

	if (Chip_TIMER_MatchPending(LPC_TIMER0, 0)) // pedals
	{
		Chip_TIMER_ClearMatch(LPC_TIMER0, 0);
		if(!EncoderDrdy)
		{
			GPIOOff(GPIO_LED_1);

			encoder_ind ++;
			LedToggle(GPIO_LED_2);

			EncoderBuff[0] = HEAD;
			EncoderBuff[1] = ENCODER_HEAD;
			EncoderBuff[2] = ENCODER_NCHAN;

			for (uint8_t ch = 0;ch < ENCODER_NCHAN; ch++)
			{
				val = encoder_ind % ENCODER_RATE + ch;// (int32_t) 8388607+16777215*(arm_sin_f32((ch+1)*(semg_ind+1)*3.1416/SEMG_RATE));
				EncoderBuff[3+ENCODER_RESOL*ch+1] = (uint8_t) (val>>8);
				EncoderBuff[3+ENCODER_RESOL*ch+2] = (uint8_t) (val);
			}

			EncoderBuff[3 + ENCODER_BUFFER_SIZE] = (uint8_t)TAIL;
			EncoderDrdy = 1;

		}
		else{
			lost++;
			GPIOOn(GPIO_LED_1);
		}
		if(!pedalsDrdy)
		{
			GPIOOff(GPIO_LED_1);

			n_pedals++;
			GPIOToggle(GPIO_LED_3);

			PedalsBuff[0] = HEAD;
			PedalsBuff[1] = PEDAL_HEAD;
			PedalsBuff[2] = PEDAL_NCHAN;

			for (uint8_t ch = 0;ch < PEDAL_NCHAN; ch++)
			{
				val = semg_ind % SEMG_RATE + ch;// (int32_t) 8388607+16777215*(arm_sin_f32((ch+1)*(semg_ind+1)*3.1416/SEMG_RATE));
				PedalsBuff[3 + PEDAL_RESOL*ch+0] = (uint8_t) (val>>16);
				PedalsBuff[3 + PEDAL_RESOL*ch+1] = (uint8_t) (val>>8);
				PedalsBuff[3 + PEDAL_RESOL*ch+2] = (uint8_t) (val);
			}
			PedalsBuff[3 + PEDAL_BUFFER_SIZE] = (uint8_t)TAIL;
			pedalsDrdy = 1;
		}
		else
		{
			lost ++;
			GPIOOn(GPIO_LED_1);
		}
	}
}


/*
 * TIMER1_IRQHandler manages the sEMG and the enconder
 * simulation and transmission using UART.
 */
void TIMER1_IRQHandler(void)
{
	int32_t UART_data, val = 0;
	static uint32_t n_pedals =0;

	static uint32_t lost = 0,n_semg = 0;
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) // sEMG
	{
		/*if(!sEMGDrdy)
		{
			n_semg++;
			//GPIOOn(GPIO_LED_1);
			Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
			uint8_t head = HEAD, semghead = SEMG_HEAD;
			sEMGBuff[0] = HEAD;
			sEMGBuff[1] = SEMG_HEAD;
			sEMGBuff[2] = SEMG_NCHAN;
			for (uint8_t ch = 0;ch < SEMG_NCHAN; ch++)
			{
				val = semg_ind % SEMG_RATE + ch;// (int32_t) 8388607+16777215*(arm_sin_f32((ch+1)*(semg_ind+1)*3.1416/SEMG_RATE));
				sEMGBuff[3+sEMG_RESOL*ch+0] = (uint8_t) (val>>16);
				sEMGBuff[3+sEMG_RESOL*ch+1] = (uint8_t) (val>>8);
				sEMGBuff[3+sEMG_RESOL*ch+2] = (uint8_t) (val);

			}
			sEMGBuff[3 + sEMG_BUFFER_SIZE] = (uint8_t)TAIL;
			sEMGDrdy = 1;
			semg_ind ++;

		}
		else{
			lost++;
			GPIOOn(GPIO_LED_2);
		}*/

	}
}

void sEMGBuffLoad(){
	static uint32_t lost = 0,n_semg = 0;

	n_semg++;
	//GPIOOn(GPIO_LED_1);
	Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
	uint8_t head = HEAD, semghead = SEMG_HEAD;
	sEMGBuff[0] = HEAD;
	sEMGBuff[1] = SEMG_HEAD;
	sEMGBuff[2] = SEMG_NCHAN;
	for (uint8_t ch = 0;ch < SEMG_NCHAN; ch++)
	{
		//val = semg_ind % SEMG_RATE + ch;// (int32_t) 8388607+16777215*(arm_sin_f32((ch+1)*(semg_ind+1)*3.1416/SEMG_RATE));
		sEMGBuff[3+sEMG_RESOL*ch+0] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>16);//(val>>16);
		sEMGBuff[3+sEMG_RESOL*ch+1] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>8);//(val>>8);
		sEMGBuff[3+sEMG_RESOL*ch+2] = (uint8_t) (channel_data[ADS1299_CHANNEL1]);//(val);

	}
	sEMGBuff[3 + sEMG_BUFFER_SIZE] = (uint8_t)TAIL;
	sEMGDrdy = 1;
	semg_ind ++;
	LedToggle(LED_RGB_B);
}

void SysInit(void)
{
	SystemCoreClockUpdate();
	Chip_SetupIrcClocking();
	LedsInit();
	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};

	UartInit(&serial_init);
	ADS1299Init();
    SystickInit(500, SystickInt);
    fpuInit();//Enable FPU

}

void SystickInt(void)
{

}

void ReadData(void)
{
	new_data = true;
}

void ConfigADS(void)
{
	/* Configura el canal 2 en modo diferencial, frecuencia de muestreo de 250Hz y el driver de pierna derecha */
	ADS1299SetChannelsToDefaultConfigForEMG();
	//ADS1299SetChannelsToDefaultConfigForECG();
	/* Cambia la ganancia a 1 para poder utilizarlo con un generador de funciones y que no sature el ADS */
	//ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL2, ADS1299_GAIN01);

	/* Datos convertidos */
	/* Indica a traves de la funcion de interrupcion ReadData que hay un dato nuevo en el vector channel_data y state.
	 * Los datos son de 24bits en complemento a 2.
	 */
	//ADS1299ActivateInt(ReadData, channel_data, state);
	ADS1299ActivateInt(sEMGBuffLoad, channel_data, state);

	/* Comienzo de la conversión continua */
	ADS1299StartStreaming();
}

/*==================[external functions definition]==========================*/
int main(void)
{
	//uint8_t uart_buffer[24] = {};
	//float32_t aux;
	union {
		float32_t ptpValue;
		uint8_t buffer[4];
	 } u_buffer;
	SysInit();
	ConfigADS();
	Configure_Timer0();
	//Configure_Timer1();
	serial_config serial_init = {SERIAL_PORT_PC, UART_BAUD_RATE, NULL};
	UartInit(&serial_init);

	uint16_t res = 0;

	while(1)
	{
		if(pedalsDrdy)
		{
			for(res = 0;res  < 4 + PEDAL_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, PedalsBuff+res);
			pedalsDrdy = 0;
		}
		if(sEMGDrdy)
		{
			for(res = 0;res  < 4 + sEMG_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, sEMGBuff+res);
			sEMGDrdy = 0;
			new_data = false;
		}
		if(EncoderDrdy)
		{
			for(res = 0;res  < 4 + ENCODER_BUFFER_SIZE; res++)
				UartSendByte(SERIAL_PORT_PC, EncoderBuff+res);
			EncoderDrdy = 0;
		}
	}
	return 0;
}
/*==================[end of file]============================================*/

