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
uint8_t HEAD = 0xCA;
uint8_t TAIL = 0xAC;

uint8_t PEDAL_HEAD = 0XCF;
uint8_t SEMG_HEAD = 0XCE;
uint8_t ENCODER_HEAD = 0XCD;

// stimulator
uint8_t STIM_CONFIG_HEAD = 0xFB;

uint8_t STIM_CONFIG_ENABLED = 0XF0;
uint8_t STIM_CONFIG_MODE = 0XF1;
uint8_t STIM_CONFIG_POLARITY = 0XF2;
uint8_t STIM_CONFIG_SOURCE= 0XF3;
uint8_t STIM_CONFIG_ZERO = 0XF4;
uint8_t STIM_CONFIG_TRIGGER = 0XF5;
uint8_t STIM_CONFIG_BUZZER = 0XF6;
uint8_t STIM_CONFIG_WIDTH = 0XF7;
uint8_t STIM_CONFIG_RECOVERY = 0XF8;
uint8_t STIM_CONFIG_DWELL = 0XF9;
uint8_t STIM_CONFIG_PERIOD = 0XFA;
uint8_t STIM_CONFIG_DEMAND = 0XFA;

// sEMG
uint8_t SEMG_GAIN_HEADER = 0XC0;



/*==================[external functions declaration]=========================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* #ifndef SERIAL_HEADERS_H_ */
