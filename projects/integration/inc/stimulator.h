/*
 * @brief DS8R STIMULATOR driver
 *
 * Copyright 2024, Esteban Osella.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
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
 *
 * ToDo:
 *  + make it multi channel.
 *
 */

/* Date: 2024-02-23 */

#ifndef _STIMULATOR_H_
#define _STIMULATOR_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "stdint.h"
#include "gpio.h"


/*==================[macros and definitions]=================================*/

#define STIM_SIM		0
#define STIM_NUMCHAN	1
#define STIM_TIMER		LPC_TIMER0
#define TRIGGER_GPIO	GPIO_1
#define stTriggerPW		1000 // trigger pulsewidth duration in us.
// Headers --> to headers file
//uint8_t STIM_CONFIG_ = 0XFA;



// =================== control structure ==================

typedef enum ENABLED{
	E_DISABLED = 1, E_ENABLED, E_NO_CHANGE
}ENABLED_T;

typedef enum MODE{
	MONOPHASIC= 1,BIPHASIC,M_NO_CHANGE = 0X07
}MODE_T;

typedef enum POLARITY{
	POSITIVE = 1,NEGATIVE,ALTERNATING,P_NO_CHANGE =0X07
}POLARITY_T;

typedef enum SOURCE{
	INTERNAL=1,EXTERNAL,S_NO_CHANGE = 0X07
}SOURCE_T;

typedef enum ZERO{
	START = 1,Z_NO_CHANGE =0X03
}ZERO_T;

typedef enum TRIGGER{
	INITIATE_TRIGGER = 1,T_NO_CHANGE = 0X03
}TRIGGER_T;

typedef enum BUZZER{
	B_ENABLED = 0x00, B_DISABLED
}BUZZER_T;

typedef struct {
	ENABLED_T Enabled;
	MODE_T Mode;
	POLARITY_T Polarity;
	SOURCE_T Source;
	ZERO_T Zero ;
	TRIGGER_T Trigger ;
	BUZZER_T Buzzer;
	int VALUE;
} DS8R_control_t;

//struct DS8R_control_t DS8R_control = {E_DISABLED,BIPHASIC,POSITIVE,
//		EXTERNAL,START,T_NO_CHANGE,B_DISABLED,0};

// ================== ds8r state strucutre ======================

typedef struct{
	DS8R_control_t DS8R_control;
	uint16_t Demand; //Controls the demand of the stimulus when internal selected. 10*mA
	uint16_t Width; // duration in us
	uint8_t Recovery; // Controls the recovery pusle duration when biphasic
					  // mode is selected. It's the percentage Amplitude to
					  // recovery pulse will have. The recovery pulse
					  // duration is automatically adjusted to ensure the
					  // pulse energy is the same as the stimulus pulse.
					  // Values 10 -100
	uint16_t Dwell; // us between pulse end and recovery start 1-990
	uint16_t CPULSE; //Number of stimulus pulses delivered since the device
				     // was ENABLED. This value will only be reset when the
					 // output is DISABLED and the ENABLED
	uint16_t COOC; //Number of out of compliance events since the device
				   // was ENABLED. This value will only be reset when the
				   // output is DISABLED and the ENABLED.
	uint16_t period;  	// stimulation period [ms]
} DS8Rstimulator_t;

DS8Rstimulator_t stimulator[STIM_NUMCHAN];

/**
 * @}
 */

/*==================[internal data declaration]==============================*/

void StimInit();
uint8_t StimEnable(uint8_t);
void StimSetBuff(uint8_t head,uint8_t up_value,uint8_t low_value);
uint8_t GetStimTrxFlag();
void SetStimTrxFlag(uint8_t);
void StimTimerConfig(uint8_t channel);
void StimReset();
void StimDisable(uint8_t channel);
uint8_t StimUpdateDemand(uint8_t channel, uint16_t demand);




#ifdef __cplusplus
}
#endif

#endif /* __STIMULATOR_H_ */
