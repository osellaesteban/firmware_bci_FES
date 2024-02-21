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

/** \brief Bare Metal driver for MMA8451 in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20180301 v0.1 initials initial version SM
 */

#include "MMA8451.h"         /* <= MMA8451 header */

bool MMA8451IsAlive( void ){

	/* Check connection */
	  uint8_t device_id=0;
	  uint8_t data_to_read_buffer;
	  data_to_read_buffer = MMA8451_REG_WHOAMI;
	  I2CRead(MMA8451_DEFAULT_ADDRESS, &data_to_read_buffer, 1, true, &device_id, 1, true );
	  if (device_id != 0x1A)
	  {
	    /* No MMA8451 detected ... return false */
	    return false;
	  }
	  return true;
}

bool MMA8451PrepareDefaultConfig( mma8451_config_t * config ){

   config->range = MMA8451_RANGE_2_G;
   config->dataRate = MMA8451_DATARATE_100_HZ;
   return (true);
}


bool MMA8451Config( mma8451_config_t config ){

   uint8_t transmit_data_buffer[2], data_to_read_buffer, rst = 0x40;

   transmit_data_buffer[0] = MMA8451_REG_CTRL_REG2;
   transmit_data_buffer[1] = 0x40;
   I2CWrite(MMA8451_DEFAULT_ADDRESS, &transmit_data_buffer, 2, true ); //reset

   while(rst & 0x40)
   {
	   data_to_read_buffer = MMA8451_REG_CTRL_REG2;
	   I2CRead(MMA8451_DEFAULT_ADDRESS, &data_to_read_buffer, 1, true, &rst, 1, true );
   }


   transmit_data_buffer[0] = MMA8451_REG_XYZ_DATA_CFG;
   transmit_data_buffer[1] = config.range;
   I2CWrite(MMA8451_DEFAULT_ADDRESS, &transmit_data_buffer, 2, true ); //Rango de medicion

   transmit_data_buffer[0] = MMA8451_REG_CTRL_REG2;
   transmit_data_buffer[1] = 0x02;
   I2CWrite(MMA8451_DEFAULT_ADDRESS, &transmit_data_buffer, 2, true ); // High Resolution

   transmit_data_buffer[0] = MMA8451_REG_CTRL_REG1;
   transmit_data_buffer[1] = 0x03;
   I2CWrite(MMA8451_DEFAULT_ADDRESS, &transmit_data_buffer, 2, true ); // Activate at max rate, low noise mode

   return (MMA8451IsAlive());
}


bool MMA8451Read( int16_t * x, int16_t * y, int16_t * z ){


   uint8_t data_to_read_buffer;

   data_to_read_buffer = MMA8451_REG_OUT_X_MSB;
   I2CRead(MMA8451_DEFAULT_ADDRESS,
            &data_to_read_buffer, 1, true,
            x, 1, true );

   data_to_read_buffer = MMA8451_REG_OUT_Y_MSB;
   I2CRead(MMA8451_DEFAULT_ADDRESS,
            &data_to_read_buffer, 1, true,
            y, 1, true );

   data_to_read_buffer = MMA8451_REG_OUT_Z_MSB;
   I2CRead(MMA8451_DEFAULT_ADDRESS,
            &data_to_read_buffer, 1, true,
            z, 1, true );


   return(MMA8451IsAlive());
}
