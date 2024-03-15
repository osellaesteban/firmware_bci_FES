/*
 * controller.c
 *
 *  Created on: 8 mar. 2024
 *      Author: osella
 */

#include "controller.h"
#include "stimulator.h"
#include "serial_headers.h"

//#include "serial_headers.h"

uint16_t angle = 0;

uint8_t ctrlTxFlag = 0;

uint8_t ctrlTxBuff[CTRL_BUFF_SIZE] = {};

limits_t limits[STIM_NUMCHAN];
fatigue_t ControlFatigue[STIM_NUMCHAN];

void ControlInitialize(){
	ctrlTxBuff[0] = HEAD;
	ctrlTxBuff[1] = CONTROL_HEAD;
	ctrlTxBuff[6] = TAIL;
	uint8_t it = 0;
	for(it = 0; it < STIM_NUMCHAN; it++)
	{
		ControlFatigue[it].LowerLimit = 0;
		ControlFatigue[it].UpperLimit = 10000;
		ControlFatigue[it].Fatigue =1;
	}
	limits[QUAD].lowLimit = 30;
	limits[QUAD].upLimit = 80;
	StimInit();
}


uint8_t* ControlGetBuffer(){
	return ctrlTxBuff;
}

void ControlSetTrxFlag(uint8_t value){
	ctrlTxFlag = value;
}

uint8_t ControlGetTrxFlag(){
	return ctrlTxFlag;
}

void ControlUpdateAngle(uint16_t val){
	angle = val;
}


uint16_t ControlComputeDemand(muscle_t muscle,uint16_t angle){

	static uint16_t demand = 0;
	if ((uint8_t) muscle < STIM_NUMCHAN){
		// Generate a specific demand computation
		demand = angle;
	}

	else{
		demand = 0;
	}

	return demand;
}

uint8_t ControlSetMuscleFatigueLimits(muscle_t muscle,uint32_t lower,uint32_t upper){
	uint8_t res = 0;
	if ((uint8_t) muscle < STIM_NUMCHAN){
		if (lower < upper){
			ControlFatigue[muscle].LowerLimit = lower;
			ControlFatigue[muscle].UpperLimit = upper;
		}
		else
			res = -1;
	}
	else
		res = -2;
	return res;

}

uint8_t ControlSetMuscleFatigue(muscle_t muscle, uint32_t value){
	uint8_t res = 0;

	if ((uint8_t) muscle < STIM_NUMCHAN){
		ControlFatigue[muscle].Fatigue = value;
	}
	else
		res = -1;
	return res;
}

uint32_t ControlGetFatigue(muscle_t muscle){
	return ControlFatigue[muscle].Fatigue;
}

uint8_t ControlGetMuscleAngleLimits(muscle_t muscle,uint16_t* lower,uint16_t* upper)
{
	uint8_t res = 0;
	if ((uint8_t) muscle < STIM_NUMCHAN){
			*upper = limits[muscle].upLimit;
			*lower = limits[muscle].lowLimit;
		}
	else
		res = -1;
	return res;
}

uint8_t ControlSetMuscleAngleLimits(muscle_t muscle,uint16_t lower,  uint16_t upper){
	uint8_t res = 0;
	if ((uint8_t) muscle < STIM_NUMCHAN){
		if (lower < upper){
			limits[muscle].upLimit = upper ;
			limits[muscle].lowLimit = lower;
			}
		else
			res = -1;
	}
	else
		res = -2;
	return res;
}

uint32_t ControlMotorUpdate(){
	static uint32_t res = 0;
	res++;
	// Generate a specific motor control output computation
	if (!ControlGetTrxFlag()){
		ctrlTxBuff[2] = CONTROL_MOTOR_HEAD;
		ctrlTxBuff[3] = res >> 8;
		ctrlTxBuff[4] = (uint8_t) res;
		ctrlTxBuff[5] = TAIL;
		ControlSetTrxFlag(1);
	}
	return res;
}

void ControlUpdateStimulation(void){
	uint16_t demand = 0;
	uint8_t it = 0;

	for (it = 0; it < STIM_NUMCHAN; it++)
	{
		if ((angle> limits[it].lowLimit) && (angle < limits[it].upLimit)){
			if ((ControlFatigue[it].Fatigue < ControlFatigue[it].UpperLimit) &&
					(ControlFatigue[it].Fatigue > ControlFatigue[it].LowerLimit)){
				demand = ControlComputeDemand(it,angle);
				StimUpdateDemand(it, demand);
				if (StimGetEnabled(it) != E_ENABLED)
					StimEnable(it);
				if (!ControlGetTrxFlag()){
					ctrlTxBuff[2] = CONTROL_STIM_HEAD;
					ctrlTxBuff[3] = it;
					ctrlTxBuff[4] = demand >> 8;
					ctrlTxBuff[5] = (uint8_t) demand;
					ControlSetTrxFlag(1);
				}
			}
		}
		else if(StimGetEnabled(it) == E_ENABLED)
			StimDisable(it);
	}
}

