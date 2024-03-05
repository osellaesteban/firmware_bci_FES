/*
 * stimulator.c
 *
 *  Created on: 2024-02-23
 *      Author: Estenam Osella
 *
 *      ToDO:
 *      - Channel selection logics
 *      - Solve the NVIC_EnableIRQ(TIMER1_IRQn); reference to the generic timer.
 *      	Also to the prescaler chip rate and the handler.
 *      -
 *
 */


#include "stimulator.h"
//#include "ADS1299.h"
#include "uart.h"
//#include "serial_headers.h"

extern uint8_t HEAD;
extern uint8_t TAIL;
//extern UART_BAUD_RATE;

uint8_t StimTrxFlag = 0;

/*==================[internal data definition]===============================*/
uint8_t StimActiveChannel = 0;

extern uint8_t STIM_CONFIG_HEAD;

extern uint8_t STIM_CONFIG_ENABLED;
extern uint8_t STIM_CONFIG_MODE;
extern uint8_t STIM_CONFIG_POLARITY;
extern uint8_t STIM_CONFIG_SOURCE;
extern uint8_t STIM_CONFIG_ZERO;
extern uint8_t STIM_CONFIG_TRIGGER;
extern uint8_t STIM_CONFIG_BUZZER;
extern uint8_t STIM_CONFIG_WIDTH;
extern uint8_t STIM_CONFIG_RECOVERY ;
extern uint8_t STIM_CONFIG_DWELL;
extern uint8_t STIM_CONFIG_PERIOD;
#define STIM_BUFF_SIZE	6 // 2 for the headers, 1 for variable, 2 for the data, 1 for the tail.

uint8_t stimTxBuff[STIM_BUFF_SIZE] = {};

// External functions definitions
uint8_t GetStimTrxFlag()
{
	return StimTrxFlag;
}
void SetStimTrxFlag(uint8_t val){
	StimTrxFlag = val;
}

void StimSetBuff(uint8_t head,uint8_t up_value,uint8_t low_value)
{
	stimTxBuff[0] = HEAD;
	stimTxBuff[1] = STIM_CONFIG_HEAD;
	stimTxBuff[2] = head;
	if (head == STIM_CONFIG_PERIOD){
		stimTxBuff[3] = up_value;
		stimTxBuff[3] = low_value;
		stimTxBuff[5] = TAIL;
	}
	else {
		stimTxBuff[3] = low_value;
		stimTxBuff[4] = TAIL;
	}
	StimTrxFlag = TRUE;
}

/*
 * Initializes the DAC
 */
void DacInit(){
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

/*
 * Timer0 event handler
 */
void TIMER0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(STIM_TIMER, 0))
	{
		Chip_TIMER_ClearMatch(STIM_TIMER, 0);// Period definition
		Chip_DAC_UpdateValue(LPC_DAC, stimulator[StimActiveChannel].Demand);
		GPIOOn(TRIGGER_GPIO);
		GPIOToggle(GPIO_LED_3);
	}
	if (Chip_TIMER_MatchPending(STIM_TIMER, 1))
	{
		Chip_TIMER_ClearMatch(STIM_TIMER, 1);
		Chip_DAC_UpdateValue(LPC_DAC, 0);
		GPIOOff(TRIGGER_GPIO);

	}
	if (Chip_TIMER_MatchPending(STIM_TIMER, 2))
	{
		Chip_TIMER_ClearMatch(STIM_TIMER, 2);
		//Chip_DAC_UpdateValue(LPC_DAC, 0);
		GPIOOff(TRIGGER_GPIO);

	}
}


void StimReset(){
	StimInit();
}

/*
 * Hardware initialization
 */
void StimInit(){
	DacInit();
    GPIOInit(TRIGGER_GPIO, GPIO_OUTPUT);
    stimulator[StimActiveChannel].period = 20;
    stimulator[StimActiveChannel].Demand = 100;
    stimulator[StimActiveChannel].Width = 500;
    StimTimerConfig(StimActiveChannel);
}

uint8_t StimEnable(uint8_t chann){

	// 4. Set appropriate value in MCR.
	Chip_TIMER_MatchEnableInt(STIM_TIMER,0);
	Chip_TIMER_MatchEnableInt(STIM_TIMER,1);
	Chip_TIMER_MatchEnableInt(STIM_TIMER,2);

	Chip_TIMER_ResetOnMatchEnable(STIM_TIMER,0);
	Chip_TIMER_Reset(STIM_TIMER);

	// 5. Enable the timer by configuring TCR.
	Chip_TIMER_Enable(STIM_TIMER);

	// 6. Enable timer interrupt if needed.
	NVIC_EnableIRQ(TIMER0_IRQn);
	Chip_DAC_UpdateValue(LPC_DAC, (uint32_t) 0);
	//NVIC_ClearPendingIRQ(TIMER1_IRQn);
	// stimulator.state = ST_UP;
	// stimInit = 1;
}

void StimDisable(uint8_t channel){
	Chip_TIMER_Disable(STIM_TIMER);
	Chip_DAC_UpdateValue(LPC_DAC, (uint32_t) 0);
	GPIOOff(TRIGGER_GPIO);

}

void StimTimerConfig(uint8_t channel){

	// 1. Reset the timer by configuring TCR.
	Chip_TIMER_Reset(STIM_TIMER);

	// 2. Set Prescale value in PR
	Chip_TIMER_PrescaleSet(STIM_TIMER,Chip_Clock_GetRate(CLK_MX_TIMER0) / 1000000 - 1);

	// 3. Set the count value in Match register(MR)
	// Match0 -> period
	Chip_TIMER_MatchEnableInt(STIM_TIMER, 0);
	Chip_TIMER_ResetOnMatchEnable(STIM_TIMER, 0);
	Chip_TIMER_StopOnMatchDisable(STIM_TIMER, 0);
	Chip_TIMER_SetMatch(STIM_TIMER, 0, (uint32_t) ( stimulator[StimActiveChannel].period*1000));

	// Match 1 -> Change from up to down
	Chip_TIMER_MatchEnableInt(STIM_TIMER, 1);
	Chip_TIMER_ResetOnMatchDisable(STIM_TIMER, 1);
	Chip_TIMER_StopOnMatchDisable(STIM_TIMER, 1);
	Chip_TIMER_SetMatch(STIM_TIMER, 1, (uint32_t) stTriggerPW );

	// Match 2 -> Change from down to zero
	Chip_TIMER_MatchEnableInt(STIM_TIMER, 2);
	Chip_TIMER_ResetOnMatchDisable(STIM_TIMER, 2);
	Chip_TIMER_StopOnMatchDisable(STIM_TIMER, 2);
	Chip_TIMER_SetMatch(STIM_TIMER, 2, (uint32_t) (stTriggerPW + stimulator[StimActiveChannel].Width));
	//StimEnable(channel);

}

uint8_t StimUpdateDemand(uint8_t channel, uint16_t demand){
	uint8_t res = 0;
	if (channel < STIM_NUMCHAN)
	{
		stimulator[channel].Demand = demand;
	}
	else
		res = 1;
	return res;
}

