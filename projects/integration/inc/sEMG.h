/*
 * sEMG.h
 *
 *  Created on: 21 feb. 2024
 *      Author: osella esteban
 *
 *
 */

#ifndef PROJECTS_INTEGRATION_INC_SEMG_H_
#define PROJECTS_INTEGRATION_INC_SEMG_H_
#include "chip.h"
#include "stdint.h"
#include "ADS1299.h"
#include "led.h"


#define SIMULATE	0

// sEMG sampling rate
#define SEMG_RATE		2000
// number of channels
#define SEMG_NCHAN 		1
// resolution in bytes
#define sEMG_RESOL		3

#define sEMG_BUFFER_SIZE	(SEMG_NCHAN*sEMG_RESOL)


uint32_t sEMGVal;


//void ReadData(void);

void sEMGGainUpdate(gain_ADS1299_t);

// void sEMGBuffLoad(void);
void ConfigADS(void);
void DeactivateADS();
void sEMGSetDRDY(uint8_t);
uint8_t*  sEMGGetBuffer();
uint16_t sEMGGetBuffSize();
uint16_t sEMGGetsEMGRate();
uint8_t sEMGGetDRDY();
int32_t GetsEMGVal();

#endif /* PROJECTS_INTEGRATION_INC_SEMG_H_ */
