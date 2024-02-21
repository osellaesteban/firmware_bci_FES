/* Copyright 2018,
 * Sebastian Mateos
 * sebastianantoniomateos@gmail.com
 * Cátedra Electrónica Programable
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

/* Date: 2018-03-01 */

#ifndef _MMA8451_H_
#define _MMA8451_H_

/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup MMA8451
 ** @{ */

/** @brief Drivers Acelerometro
**
**/

/*==================[inclusions]=============================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
#define MMA8451_DEFAULT_ADDRESS                 (0x1C)    // if A is GND, its 0x1C
/*=========================================================================*/

#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_OUT_Y_MSB     0x03
#define MMA8451_REG_OUT_Z_MSB     0x05
#define MMA8451_REG_SYSMOD        0x0B
#define MMA8451_REG_WHOAMI        0x0D
#define MMA8451_REG_XYZ_DATA_CFG  0x0E
#define MMA8451_REG_PL_STATUS     0x10
#define MMA8451_REG_PL_CFG        0x11
#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B
#define MMA8451_REG_CTRL_REG4     0x2D
#define MMA8451_REG_CTRL_REG5     0x2E



#define MMA8451_PL_PUF            0
#define MMA8451_PL_PUB            1
#define MMA8451_PL_PDF            2
#define MMA8451_PL_PDB            3
#define MMA8451_PL_LRF            4
#define MMA8451_PL_LRB            5
#define MMA8451_PL_LLF            6
#define MMA8451_PL_LLB            7


/*==================[typedef]================================================*/

typedef enum
{
 MMA8451_RANGE_8_G           = 0b10,   // +/- 8g
 MMA8451_RANGE_4_G           = 0b01,   // +/- 4g
 MMA8451_RANGE_2_G           = 0b00    // +/- 2g (default value)
} mma8451_range_t;


/* Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
 MMA8451_DATARATE_800_HZ     = 0b000, //  800Hz
 MMA8451_DATARATE_400_HZ     = 0b001, //  400Hz
 MMA8451_DATARATE_200_HZ     = 0b010, //  200Hz
 MMA8451_DATARATE_100_HZ     = 0b011, //  100Hz
 MMA8451_DATARATE_50_HZ      = 0b100, //   50Hz
 MMA8451_DATARATE_12_5_HZ    = 0b101, // 12.5Hz
 MMA8451_DATARATE_6_25HZ     = 0b110, // 6.25Hz
 MMA8451_DATARATE_1_56_HZ    = 0b111, // 1.56Hz

 MMA8451_DATARATE_MASK       = 0b111
} mma8451_dataRate_t;

typedef struct{
   mma8451_dataRate_t    dataRate;    /* Data Output Rate Bits. These bits set the rate at which data
                                * is written to all three data output registers.*/
   mma8451_range_t 		 range; /* Range Configuration Bits.*/
} mma8451_config_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

bool MMA8451IsAlive(void);
bool MMA8451PrepareDefaultConfig( mma8451_config_t * config );
bool MMA8451Config( mma8451_config_t config );
bool MMA8451Read( int16_t * x, int16_t * y, int16_t * z );

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _MMA8451_H_ */
