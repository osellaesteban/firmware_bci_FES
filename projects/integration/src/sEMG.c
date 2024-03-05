/*
 * sEMG.c
 *
 *  Created on: 21 feb. 2024
 *      Author: osella
 */


#include "sEMG.h"

//#include "systick.h"
extern uint8_t HEAD;
extern uint8_t SEMG_HEAD;
extern uint8_t TAIL;

//extern uint8_t PEDAL_HEAD;

//extern uint8_t ENCODER_HEAD;
gain_ADS1299_t sEMGGain = ADS1299_GAIN01;
int32_t channel_data[8];
uint8_t state[3];



// samples obtained and loaded to the buffer
uint8_t sEMGDrdy = 0;
static uint8_t sEMGBuff[sEMG_BUFFER_SIZE+4] = {};

uint8_t*  sEMGGetBuffer(){
	return sEMGBuff;
}

uint16_t sEMGGetBuffSize(){
	return sEMG_BUFFER_SIZE;
}
uint16_t sEMGGetsEMGRate(){
	return SEMG_RATE;
}

uint8_t sEMGGetDRDY(){
	return (sEMGDrdy);
}
int32_t GetsEMGVal(){
	return sEMGVal;
}

void sEMGBuffLoad(){
	//static uint32_t lost = 0,n_semg = 0;

	static uint32_t semgmax = 0,semgmin = 2^24;
	//GPIOOn(GPIO_LED_1);
	Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
	//uint8_t head = HEAD, semghead = SEMG_HEAD;
	sEMGBuff[0] = HEAD;
	sEMGBuff[1] = SEMG_HEAD;
	sEMGBuff[2] = SEMG_NCHAN;
	sEMGVal = channel_data[ADS1299_CHANNEL1];//
	if (sEMGVal>semgmax)
		semgmax = sEMGVal;
	if (sEMGVal<semgmin)
		semgmin = sEMGVal;
	for (uint8_t ch = 0;ch < SEMG_NCHAN; ch++)
	{
		//val = semg_ind % SEMG_RATE + ch;// (int32_t) 8388607+16777215*(arm_sin_f32((ch+1)*(semg_ind+1)*3.1416/SEMG_RATE));
		sEMGBuff[3+sEMG_RESOL*ch+0] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>16);//(val>>16); //channel_data[ADS1299_CHANNEL1]
		sEMGBuff[3+sEMG_RESOL*ch+1] = (uint8_t) (channel_data[ADS1299_CHANNEL1]>>8);//(val>>8); //channel_data[ADS1299_CHANNEL1]
		sEMGBuff[3+sEMG_RESOL*ch+2] = (uint8_t) (channel_data[ADS1299_CHANNEL1]);//(val);//channel_data[ADS1299_CHANNEL1]

	}
	sEMGBuff[3 + sEMG_BUFFER_SIZE] = (uint8_t)TAIL;
	sEMGDrdy = 1;
	semg_ind ++;
	LedToggle(LED_RGB_B);
}


void sEMGGainUpdate(gain_ADS1299_t newGain){
	sEMGGain  = newGain;
	ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1, sEMGGain );
}

/*
 * TIMER1_IRQHandler manages the sEMG and the enconder
 * simulation and transmission using UART.
 */
void TIMER1_IRQHandler(void)
{
	int32_t val = 0;
	//static uint32_t n_pedals =0;

	static uint32_t lost = 0,n_semg = 0;
	if (Chip_TIMER_MatchPending(LPC_TIMER1, 0)) // sEMG
	{
		if(!sEMGDrdy)
		{
			n_semg++;
			//GPIOOn(GPIO_LED_1);
			Chip_TIMER_ClearMatch(LPC_TIMER1, 0);
			//uint8_t head = HEAD;// semghead = SEMG_HEAD;
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
		}

	}
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

void ConfigADS(void)
{
	if (!SIMULATE){
		ADS1299Init();

		/* Configura el canal 2 en modo diferencial, frecuencia de muestreo de 250Hz y el driver de pierna derecha */
		ADS1299SetChannelsToDefaultConfigForEMG();
		//ADS1299SetChannelsToDefaultConfigForECG();
		/* Cambia la ganancia a 1 para poder utilizarlo con un generador de funciones y que no sature el ADS */
		ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1, sEMGGain );

		/* Datos convertidos */
		/* Indica a traves de la funcion de interrupcion ReadData que hay un dato nuevo en el vector channel_data y state.
		 * Los datos son de 24bits en complemento a 2.
		 */
		//ADS1299ActivateInt(ReadData, channel_data, state);
		ADS1299ActivateInt(sEMGBuffLoad, channel_data, state);

		/* Comienzo de la conversión continua */
		ADS1299StartStreaming();
	}
	else{
		Configure_Timer1();
	}
}

void DeactivateADS(){
	if(!SIMULATE)
	{
		ADS1299DeactivateInt();
	}
	else
		Chip_TIMER_Disable(LPC_TIMER1);
}

void sEMGSetDRDY(uint8_t val){
	sEMGDrdy = val;
}


