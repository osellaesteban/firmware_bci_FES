/*
 * controller.h
 *
 *  Created on: 8 mar. 2024
 *      Author: Esteban Osella
 *
 *      take care not to be writing the buffer while it is being filled.
 */

#ifndef PROJECTS_INTEGRATION_INC_CONTROLLER_H_
#define PROJECTS_INTEGRATION_INC_CONTROLLER_H_

#include "stdint.h"
#include "gpio.h"


#define FATIGUE_STOP_LOW 10
#define FATIGUE_STOP_UP 20
#define CTRL_BUFF_SIZE 7 // HEAD, CTRL HEAD, MOTOR/STIM HEAD,DATA1/CHAN, DATA2/DATA1,TAIL/DATA2,NULL/TAIL
//extern STIM_NUMCHAN;

// Data type definitions
typedef enum {QUAD, GLUTEI, HAMSTRINGS, GASTROCNEMIUS} muscle_t;

typedef struct {
	uint16_t lowLimit;
	uint16_t upLimit;
}limits_t;

typedef struct {
	uint32_t UpperLimit;
	uint32_t LowerLimit;
	uint32_t Fatigue;
} fatigue_t;


// ---------




// ------------------- External functions-------------------


/*
 * Update the angle value used by the controller.
 */
void ControlUpdateAngle(uint16_t val);

/*
 * Initilizes buffer deffinitions and variables.
 * Also determines the starting angular and fatigue limits.
 * Initilizes the stimulator
 */
void ControlInitialize();

/*
 * Returns the pointer to the buffer's first element
 */
uint8_t* ControlGetBuffer();

/*
 * Operates the buffer flag
 */
void ControlSetTrxFlag(uint8_t);

/*
 * Returns the buffer flag
 */
uint8_t ControlGetTrxFlag();

/*
 * Returns the lastly computed fatigue value
 */
uint32_t ControlGetFatigue(muscle_t);

/*
 * Sets a particular muscle fatigue limits. Returns:
 * 0 if no problems where detected.
 * -1 if the referred muscle overheads the muscles array
 * -2 if the lower limit is over the upper limit
 */
uint8_t ControlSetMuscleFatigueLimits(muscle_t muscle,uint32_t lower,uint32_t upper);

/*
 * Determines the muscle fatigue
 */
uint8_t ControlSetMuscleFatigue(muscle_t muscle, uint32_t fatigue);

/*
 * Returns the lower and upper limits muscular stimulation angular limits
 */
uint8_t ControlGetMuscleAngleLimits(muscle_t muscle, uint16_t* lower, uint16_t* upper);

/*
 * Determines the lower and upper start angular muscle activation for the referred
 * muscle
 */
uint8_t ControlSetMuscleAngleLimits(muscle_t muscle, uint16_t lower, uint16_t upper);

/*
 * Updates the motor output parameter
 */
uint32_t ControlMotorUpdate();

/*
 * Updates the stimulation parameters.
 */
void ControlUpdateStimulation(void);


#endif /* PROJECTS_INTEGRATION_INC_CONTROLLER_H_ */
