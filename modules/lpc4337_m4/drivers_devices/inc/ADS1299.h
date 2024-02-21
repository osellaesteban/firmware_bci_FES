/* Copyright 2018,
 * Eduardo Filomena
 * efilomena@bioingenieria.uner.edu.ar
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
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
/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */

/** @addtogroup ADS1299
 *  @{
 *  @file ADS1299.h
 *  @brief Driver Bare Metal
 *
 *  @section changeLog
 *
 *  v1.1 Initial version EF
 *  v1.2 Modifications for less RAM consumption SM
 *
 *  Initials   |     Name
 * :----------:|:----------------
 *  SM		   | Sebastian Mateos
 *  EF		   | Eduardo Filomena
 *
 *  @date 09/11/2018
 */

#ifndef ADC1299_H
#define ADC1299_H

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "bool.h"
#include "gpio.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/
/** @typedef enum comand_ADS1299_t
 * @brief SPI commands of ADS1299
 **/
typedef enum
{
	ADS1299_WAKEUP = 0x02, // Wake-up from standby mode
	ADS1299_STANDBY = 0x04, // Enter Standby mode
	ADS1299_RESET = 0x06, // Reset the device registers to default
	ADS1299_START = 0x08, // Start and restart (synchronize) conversions
	ADS1299_STOP = 0x0A, // Stop conversion
	ADS1299_RDATAC = 0x10, // Enable Read Data Continuous mode (default mode at power-up)
	ADS1299_SDATAC = 0x11, // Stop Read Data Continuous mode
	ADS1299_RDATA = 0x12 // Read data by command; supports multiple read back

}comand_ADS1299_t;

/** @typedef enum register_ADS1299_t
 * @brief Registers of ADS1299
 **/
typedef enum
{
   ADS1299_ADS_ID = 0x3E,	// product ID for ADS1299
   ADS1299_ID_REG = 0x00,	// this register contains ADS_ID
   ADS1299_CONFIG1,
   ADS1299_CONFIG2,
   ADS1299_CONFIG3,
   ADS1299_LOFF,
   ADS1299_CH1SET,
   ADS1299_CH2SET,
   ADS1299_CH3SET,
   ADS1299_CH4SET,
   ADS1299_CH5SET,
   ADS1299_CH6SET,
   ADS1299_CH7SET,
   ADS1299_CH8SET,
   ADS1299_BIAS_SENSP,
   ADS1299_BIAS_SENSN,
   ADS1299_LOFF_SENSP,
   ADS1299_LOFF_SENSN,
   ADS1299_LOFF_FLIP,
   ADS1299_LOFF_STATP,
   ADS1299_LOFF_STATN,
   ADS1299_GPIO,
   ADS1299_MISC1,
   ADS1299_MISC2,
   ADS1299_CONFIG4

}register_ADS1299_t;

/** @typedef enum pin_ADS1299_t
 * @brief Pins of ADS1299
 **/
typedef enum
{
	ADS1299_PWDN_PIN = GPIO_LCD_2,
	ADS1299_DRDY_PIN = GPIO_LCD_3,
	ADS1299_RESET_PIN = GPIO_LCD_4,
	ADS1299_CS_PIN = GPIO_LCD_EN,
	ADS1299_START_PIN = GPIO_LCD_RS
}pin_ADS1299_t;

/** @typedef enum channel_ADS1299_t
 * @brief Channels of ADS1299
 **/
typedef enum
{
	ADS1299_CHANNEL1 =	0x00,
	ADS1299_CHANNEL2,
	ADS1299_CHANNEL3,
	ADS1299_CHANNEL4,
	ADS1299_CHANNEL5,
	ADS1299_CHANNEL6,
	ADS1299_CHANNEL7,
	ADS1299_CHANNEL8
}channel_ADS1299_t;

/** @typedef enum frec_ADS1299_t
 * @brief Sampling frecs of ADS1299
 **/
typedef enum
{
	ADS1299_SAMPLE_RATE_16kHZ =	0x00,  //(b00000000)
	ADS1299_SAMPLE_RATE_8kHZ,  //(b00000001)
	ADS1299_SAMPLE_RATE_4kHZ,  //(b00000010)
	ADS1299_SAMPLE_RATE_2kHZ,  //(b01000011)
	ADS1299_SAMPLE_RATE_1kHZ,  //(b00000100)
	ADS1299_SAMPLE_RATE_500HZ, //(b00000101)
	ADS1299_SAMPLE_RATE_250HZ  //(b00000110)

}frec_ADS1299_t;

/** @typedef enum gain_ADS1299_t
 * @brief Channel gains of ADS1299
 **/
typedef enum
{
   ADS1299_GAIN01 = (0b00000000),	// 0x00
   ADS1299_GAIN02 = (0b00010000),	// 0x10
   ADS1299_GAIN04 = (0b00100000),	// 0x20
   ADS1299_GAIN06 = (0b00110000),	// 0x30
   ADS1299_GAIN08 = (0b01000000),	// 0x40
   ADS1299_GAIN12 = (0b01010000),	// 0x50
   ADS1299_GAIN24 = (0b01100000)	// 0x60

}gain_ADS1299_t;

/** @typedef enum loff_ADS1299_t
 * @brief Lead off modes of ADC1299
 **/
typedef enum
{
	ADS1299_LOFF_DC =	0x00,
	ADS1299_LOFF_AC,
}loff_ADS1299_t;

/** @typedef enum inCod_ADS1299_t
 * @brief Lead off modes of ADC1299
 **/
typedef enum
{
	ADS1299_NORMAL = 0x00, // (0b00000000)
	ADS1299_SHORTED, // (0b00000001)
	ADS1299_BIAS_MEAS, // (0b00000010)
	ADS1299_MVDD, // (0b00000011)
	ADS1299_TEMP, // (0b00000100)
	ADS1299_TESTSIG, // (0b00000101)
	ADS1299_BIAS_DRP, // (0b00000110)
	ADS1299_BIAL_DRN // (0b00000111)
}in_cod_ADS1299_t;

/** @typedef struct CHnSet_ADS1299_t
 * @brief Configure channel options of ADS1299
 **/
typedef struct
{
	in_cod_ADS1299_t channel_input;
	bool use_srb2;
	gain_ADS1299_t pga_gain;
	bool power_down;
}CHnSet_ADS1299_t;


/** @typedef enum test_amp_ADS1299_t
 * @brief Configure test signal option of ADS1299
 **/
typedef enum
{
    ADS1299_ADSTESTSIG_AMP_1X = (0b00000000),
    ADS1299_ADSTESTSIG_AMP_2X = (0b00000100)
}test_amp_ADS1299_t;

/** @typedef enum test_type_ADS1299_t
 * @brief Configure test signal option of ADS1299
 **/
typedef enum
{
    ADS1299_ADSTESTSIG_PULSE_SLOW = (0b00000000),
    ADS1299_ADSTESTSIG_PULSE_FAST = (0b00000001),
    ADS1299_ADSTESTSIG_DCSIG = (0b00000011)
}test_type_ADS1299_t;
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
/**
 * @brief  Configure pines connected to the ADS and the ADS itself. Hard Reset ADS and power up sequence
 */
void ADS1299Init(void);

/**
 * @brief Start continuous data acquisition
 */
void ADS1299StartStreaming(void);

/**
 * @brief Send data via UART FTDI in OpenBCI format
 */
void ADS1299ActivateUARTStreaming(void);

/**
 * @brief Deactivate send data via UART FTDI
 */
void ADS1299DeactivateUARTStreaming(void);

/**
 * @brief Configure data acquisition with interrupt
 * @param[in] *ptr_int_func Pointer to function thats indicates new data in buffer
 * @param[in] *ptr_channel_data 8 elements vector to save data raw
 * @param[in] *ptr_state_ext 3 element vector to save state of ADC
 */
void ADS1299ActivateInt(void * ptr_int_func, int32_t * ptr_channel_data, uint8_t * ptr_state_ext);

/**
 * @brief Deactivate data acquisition with interrupt
 */
void ADS1299DeactivateInt(void);

/**
 * @brief Data acquisition
 */
void ADS1299GetData(void);

/**
 * @brief Data acquisition
 * @param[in] *uart_buffer 24 elements vector to send data via UART FTDI in OpenBCI format
 */
void ADS1299SendUART(uint8_t *uart_buffer);

/**
 * @brief Stop continuous data acquisition
 */
void ADS1299StopStreaming(void);

/**
 * @brief Send SPI command to ADS
 * @param[in] command Command to send through SPI @ref command_ADS1299_t
 */
void ADS1299SendCommand(comand_ADS1299_t command);

/**
 * @brief Write one ADS register
 * @param register_ADS1299_t Register address to write
 * @param value Byte to write in register
 */
void ADS1299WriteRegister(register_ADS1299_t address,uint8_t value);

/**
 * @brief Read one ADS register
 * @param address Register address to read
 * @return Requested register value
 */
uint8_t ADS1299ReadRegister(register_ADS1299_t address);

/**
 * @brief Change pin state to high
 * @param pin Pin to change state (DRDY, RST, START, CS, PWDN)
 */
void ADS1299PinHigh(pin_ADS1299_t pin);

/**
 * @brief Change pin state to low
 * @param pin Pin to change state (DRDY, RST, START, CS, PWDN)
 */
void ADS1299PinLow(pin_ADS1299_t pin);

/**
 * @brief Change pin state of CS pin
 * @param state State of CS
 */
void ADS1299CS(uint8_t state);

/**
 * @brief Set all channels with the default values to ECG data adquisition
 */
void ADS1299SetChannelsToDefaultConfigForECG(void);

/**
 * @brief Set all channels with the default values to EEG data adquisition
 */
void ADS1299SetChannelsToDefaultConfigForEEG(void);

/**
 * @brief Set all channels with the default values to EMG data adquisition
 */
void ADS1299SetChannelsToDefaultConfigForEMG(void);
/**
 * @brief Write settings of all channels
 * @param[in] channel Channel to be config @ref channel_t
 * @param[in] config Info to configure channel @ref CHnSet_ADS1299_t
 */
void ADS1299ConfigOneChannel(channel_ADS1299_t channel, CHnSet_ADS1299_t config);

/**
 * @brief Write settings of all channels
 * @param[in] *config Pointer to a 8 elements vector of @ref CHnSet_ADS1299_t
 */
void ADS1299ConfigAllChannels(CHnSet_ADS1299_t *config);

/**
 * @brief Write settings of all channels
 * @param[in] config Info to configure all channel @ref CHnSet_ADS1299_t
 */
void ADS1299ConfigAllChannelsEqual(CHnSet_ADS1299_t config);

/**
 * @brief Change the sampling frecuency
 * @param[in] frec Frec sampling is defined in @ref frec_ADS1299_t
 */
void ADS1299ChangeSamplingFrecuency(frec_ADS1299_t frec);

/**
 * @brief Change the source of signal connected to one input of the ADS1299
 * @param[in] channel Channel to change input type
 * @param[in] input_code Input types
 */
void ADS1299ChangeChannelInputType(channel_ADS1299_t channel, in_cod_ADS1299_t input_code);

/**
 * @brief Change the source of signal connected to the inputs of the ADS1299
 * @param[in] input_code Pointer to a 8 elements vector of Input types
 */
void ADS1299ChangeAllChannelInputType(in_cod_ADS1299_t *input_code);

/**
 * @brief Change the source of signal connected to the inputs of the ADS1299
 * @param[in] input_code Input types
 */
void ADS1299ChangeAllChannelInputTypeEqual(in_cod_ADS1299_t input_code);

/**
 * @brief Change one channel PGA gain.
 * @param[in] channel Channel is defined in @ref channel_ADS1299_t
 * @param[in] gain Gain is defined in @ref gain_ADS1299_t
 */
void ADS1299ChangeChannelPGAGain(channel_ADS1299_t channel, gain_ADS1299_t gain);

/**
 * @brief Change all channel PGA gain.
 * @param[in] gain Pointer to a 8 elements vector of @ref gain_ADS1299_t
 */
void ADS1299ChangeAllChannelPGAGain(gain_ADS1299_t *gain);

/**
 * @brief Change all channel PGA gain.
 * @param[in] gain Gain is defined in @ref gain_ADS1299_t
 */
void ADS1299ChangeAllChannelPGAGainEqual(gain_ADS1299_t gain);

/**
 * @brief Activate specific channel
 * @param channel Channel to activate @ref channel_ADS1299_t
 */
void ADS1299ActivateChannel(channel_ADS1299_t channel);

/**
 * @brief De-activate specific channel
 * @param channel Channel to de-activate @ref channel_ADS1299_t
 */
void ADS1299DeactivateChannel(channel_ADS1299_t channel);

/**
 * @brief Activate all channel
 */
void ADS1299ActivateAllChannels(void);

/**
 * @brief De-activate all channel
 */
void ADS1299DeactivateAllChannels(void);

/**
 * @brief Activate all bias to channels active
 */
void ADS1299ConfigBiasDrive(void);

/**
 * @brief Query to see if data is available from the ADS1299
 * @return TRUE is data is available, false otherwise
 */
bool ADS1299IsDataReady(void);

/**
 * @brief Read data one-shot
 * @param[in] *buffer 24 elements vector to save data raw
 * @param[in] *state 3 element vector to save state of ADC, uint8_t *state
 */
void ADS1299OneShotConversion(void);

/**
 * @brief Convert 8 channel data 3 bytes to 4 bytes 2nd complement
 * @param[in] *buffer 24 element vector with data to convert.
 * @param[in] *vector Vector of 8 elements in which the data will be saved.
 */
void ASD1299ConvertDataTo2ndComplement(int32_t *vector);

/**
 * @brief Configure the test signals that can be inernally generated
 * @param type Frecuence of the test signal
 * @param amp Amplitude of the test signal
*/
void ADS1299ConfigTestSignal(test_type_ADS1299_t type, test_amp_ADS1299_t amp);

/**
 * @brief Sense the lead off of all channels
 * @param[in] mode Indicates the mode of lead off detection @ref loff_t
 * @return A mask that specificates which channels are lead off
 */
uint8_t ADS1299LeadOffDetection(loff_ADS1299_t mode);

/**
 * @brief Set master GPIO state
 * @param[in] gpio GPIO to be setted
 */
void ADS1299SetGPIO(uint8_t gpio);

/**
 * @brief Get master GPIO state
 * @return GPIO state
 */
uint8_t ADS1299GetGPIO(void);
/*==================[end of file]============================================*/
#endif /* ADC1299_H */

/** @}*/
/** @}*/
/** @} doxygen end group definition */


