/* Copyright 2024,
 * Esteban Osella
 * esteban.osella@uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 * All rights reserved.
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

#ifndef SERIAL_HEADERS_H_
#define SERIAL_HEADERS_H_

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

// Headers definition
#define HEAD			0xCA
#define TAIL 			0xAC

#define PEDAL_HEAD 		0XCF
#define SEMG_HEAD 		0XCE
#define ENCODER_HEAD	0XCD

// stimulator
#define STIM_CONFIG_HEAD 		0xFB

#define STIM_CONFIG_ENABLED 	0XF0
#define STIM_CONFIG_MODE 		0XF1
#define STIM_CONFIG_POLARITY 	0XF2
#define STIM_CONFIG_SOURCE 		0XF3
#define STIM_CONFIG_ZERO 		0XF4
#define STIM_CONFIG_TRIGGER 	0XF5
#define STIM_CONFIG_BUZZER 		0XF6
#define STIM_CONFIG_WIDTH 		0XF7
#define STIM_CONFIG_RECOVERY  	0XF8
#define STIM_CONFIG_DWELL  		0XF9
#define STIM_CONFIG_PERIOD  	0XFA
#define STIM_CONFIG_DEMAND 		0XFA
#define STIM_BUFF_SIZE			6 // 2 for the headers, 1 for variable, 2 for the data, 1 for the tail.

// sEMG
#define SEMG_GAIN_HEADER 		0XC0

// FATIGUE
#define FATIGUE_HEAD    		0XD0

// COMMAND
#define COMMAND_HEAD    		0XA0
#define COMMAND_START   		0XAF
#define COMMAND_STOP   	 		0XAE
#define COMMAND_RESEND  		0XA1
#define COMMAND_ANGLE_BIAS		0XAC

// CONTROL ACTION COMPUTATION

#define CONTROL_HEAD			0XE0
#define CONTROL_MOTOR_HEAD		0XE1
#define CONTROL_STIM_HEAD		0XE2


/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef SERIAL_HEADERS_H_ */
