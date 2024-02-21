/* Copyright 2018,
 * Eduardo Filomena
 * efilomena@bioingenieria.uner.edu.ar
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
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

#include <stdint.h>

#include "ADS1299.h"
#include "bool.h"
#include "chip.h"
//#include "uart.h"
#include "spi.h"
#include "stopwatch.h"

#define BIT_RATE 8000000 /**< Bits por segundo de la SPI*/
//#define BAUD_RATE 115200 // 576000// 921600 /**< Bits por segundo de la UART*/
#define T_POR 150 /* 150 ms to allow time for the supply voltages to reach their final value. */
#define T_RST 1 /* 1us are 2 tCLK for reset pulse width */
#define T_AFTER_RST 9 /* 9us are 18 tCLK to start using the ADS1299 */
#define T_SPI 3 /* 3us are 6 tCLK to wait after the serial communication */
#define LOW_EDGE 0 /*Low edge for drdy interrupt*/
#define ALL_CHANNELS 8 /*Total number of channels for register reading*/
#define NUM_REG_STATE 3 /*Total number of state register reading*/
#define SIGN_BIT (1<<23) /*Bitmask 24bits for negative number detection*/

static bool is_running = false, int_activ = false, uart_activ = false; /**< Is streaming activated */
static void (*ptr_int_func_saved)(); /**< Pointer to external function */
static int32_t * ptr_channel_data_saved ; /**< Pointer to external channel data array */
static uint8_t * ptr_state_ext_saved ; /**< Pointer to external state array */
static uint8_t channel_activ = 0x00; /**< Indicates which channels are active for BIAS configure*/
static uint8_t sample_count = 0;

uint8_t buffer[24]; /**< Pointer to an external data buffer*/
uint8_t state[3]; /**< Pointer to an external state data arrow*/

/**
 * @brief Called by an interrupt of DRDY pin and takes the data from ADS1299
 */
void ADS1299DRDYInt(void);

/**
 * @brief SPI communication method
 * @param tx Byte to transfer
 * @return Byte received from SPI target
 */
uint8_t ADS1299SendSPI(uint8_t tx);

void ADS1299Init(void)
{
    StopWatch_Init();
    GPIOInit(ADS1299_CS_PIN,  GPIO_OUTPUT);
    GPIOInit(ADS1299_RESET_PIN, GPIO_OUTPUT);
    GPIOInit(ADS1299_START_PIN, GPIO_OUTPUT);
    GPIOInit(ADS1299_PWDN_PIN, GPIO_OUTPUT);
    GPIOInit(ADS1299_DRDY_PIN, GPIO_INPUT);

	ADS1299PinHigh(ADS1299_CS_PIN);
	ADS1299PinHigh(ADS1299_RESET_PIN);
	ADS1299PinLow(ADS1299_START_PIN); //Tie the START pin low to control conversions by command.
	ADS1299PinHigh(ADS1299_PWDN_PIN);

//	spiConfig_t spi_config = {SPI_1, MASTER, MODE1, BIT_RATE, SPI_POLLING, ADS1299CS };
	spiConfig_t spi_config = {SPI_1, MASTER, MODE1, BIT_RATE, SPI_POLLING, NULL };
	SpiInit(spi_config);

	//serial_config serial_init = {SERIAL_PORT_PC, BAUD_RATE, NULL};
	//UartInit(&serial_init);
//	InitRingBuffer(UART_USB);

	StopWatch_DelayMs(T_POR); // Recommended power up sequence requiers >Tpor (aprox. 150mS)
	ADS1299PinLow(ADS1299_RESET_PIN);
	StopWatch_DelayUs(T_RST);
	ADS1299PinHigh(ADS1299_RESET_PIN);
	StopWatch_DelayUs(T_AFTER_RST);

	ADS1299SendCommand(ADS1299_RESET); //reset the on-board ADS registers and stop DataContinousMode
	ADS1299SendCommand(ADS1299_SDATAC);
	GPIOActivInt(GPIOGP4, ADS1299_DRDY_PIN, ADS1299DRDYInt, LOW_EDGE);
	is_running = false;
	int_activ = false;
	uart_activ = false;
}

void ADS1299StartStreaming(void)
{
	ADS1299SendCommand(ADS1299_RDATAC); // enter Read Data Continuous mode
	ADS1299SendCommand(ADS1299_START);  // start the data acquisition
	is_running = true;
}
/*
void ADS1299ActivateUARTStreaming(void)
{
	uart_activ = true;
}

void ADS1299DeactivateUARTStreaming(void)
{
	uart_activ = false;
}
*/
void ADS1299ActivateInt(void * ptr_int_func, int32_t * ptr_channel_data, uint8_t * ptr_state_ext)
{
	ptr_int_func_saved = ptr_int_func;
	ptr_channel_data_saved = ptr_channel_data;
	ptr_state_ext_saved = ptr_state_ext;
	int_activ = true;
}

void ADS1299DeactivateInt(void)
{
	int_activ = false;
}

void ADS1299DRDYInt(void)
{
	uint8_t i;
	ADS1299GetData();
	sample_count++;
	if(uart_activ)
	{
		//ADS1299SendUART(buffer);
	}
	if(int_activ)
	{
		for(i=0; i<NUM_REG_STATE; i++)
		{
			ptr_state_ext_saved[i] = state[i];//read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
		}
		/* Convierte datos en complemento a 2 de 24bits a complemento a 2 en 32bits */
		for(i=0 ; i<ALL_CHANNELS ; i++)
		{
			ptr_channel_data_saved[i] = (buffer[(3*i)]<<16) | (buffer[(3*i)+1] << 8) | buffer[(3*i)+2];
			if(ptr_channel_data_saved[i] & SIGN_BIT)
			{
				ptr_channel_data_saved[i] |= 0xFF000000;
			}
		}
		ptr_int_func_saved();
	}
}

void ADS1299GetData(void)
{
	ADS1299PinLow(ADS1299_CS_PIN); //open SPI
	uint8_t i;
	for(i=0; i<3; i++) // read in the new channel data
		state[i] = ADS1299SendSPI(0x00);//read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	for(i=0; i<24; i++)
		buffer[i] = ADS1299SendSPI(0x00); // Store data RAW
	ADS1299PinHigh(ADS1299_CS_PIN); //assert HIGH the chip select pin of ADS //close SPI

}
/*
void ADS1299SendUART(uint8_t *uart_buffer)
{
//	uint8_t i, UART_data[33];
//	UART_data[0] = 0xA0;
//	UART_data[1] = sample_count;
//	for(i=0 ; i<24 ; i++)
//		UART_data[i+2] = uart_buffer[i];
//	for(i=0 ; i<6 ; i++)
//		UART_data[i+26] = 0x00;
//	UART_data[32] = 0xC0;
//	UARTSendRingBuffer(UART_USB, UART_data, 33);
	//TEST
	uint8_t i, UART_data;
	UART_data = 0xA0;
	UartSendByte(SERIAL_PORT_PC, &UART_data);
	UartSendByte(SERIAL_PORT_PC, &sample_count);
	for(i=0 ; i<24 ; i++)
		UartSendByte(SERIAL_PORT_PC, uart_buffer+i);
	UART_data = 0x00;
	for(i=0 ; i<6 ; i++)
		UartSendByte(SERIAL_PORT_PC, &UART_data);
	UART_data = 0xC0;
	UartSendByte(SERIAL_PORT_PC, &UART_data);

}*/

void ADS1299StopStreaming(void)
{
	ADS1299SendCommand(ADS1299_STOP);     // stop the data acquisition
	ADS1299SendCommand(ADS1299_SDATAC);   // stop Read Data Continuous mode to communicate with ADS
	is_running = false;
}

void ADS1299SendCommand(comand_ADS1299_t command)
{
	ADS1299PinLow(ADS1299_CS_PIN);
	ADS1299SendSPI(command);
	StopWatch_DelayUs(T_SPI); // After the serial communication is finished, always wait four or more tCLK cycles before taking CS high

	ADS1299PinHigh(ADS1299_CS_PIN);
	StopWatch_DelayUs(T_AFTER_RST); //must wait 18 tCLK cycles to execute reset command (see Datasheet, pag 35)
}

void ADS1299WriteRegister(register_ADS1299_t address,uint8_t value)
{
	if(is_running)
	{
		ADS1299StopStreaming();
		is_running = true;
	}
	uint8_t opcode = address + 0x40; //  WREG expects 010rrrrr where rrrrr = _address
	ADS1299PinLow(ADS1299_CS_PIN); //assert LOW the chip select pin of ADS //open SPI
	ADS1299SendSPI(opcode);
	ADS1299SendSPI(0x00); //send 0x00 to write only the register located at _address
	ADS1299SendSPI(value); //write the value to the register
	StopWatch_DelayUs(T_SPI); // After the serial communication is finished, always wait four or more tCLK cycles before taking CS high

	ADS1299PinHigh(ADS1299_CS_PIN); //assert HIGH the chip select pin of ADS //close SPI
	if(is_running)
		ADS1299StartStreaming();
}

uint8_t ADS1299ReadRegister(register_ADS1299_t address)
{
	if(is_running)
	{
		ADS1299StopStreaming();
		is_running = true;
	}
	uint8_t opcode = address + 0x20; //  RREG expects 001rrrrr where rrrrr = _address
	ADS1299PinLow(ADS1299_CS_PIN); //assert LOW the chip select pin of ADS //open SPI
	ADS1299SendSPI(opcode); //
	ADS1299SendSPI(0x00); //send 0x00 to read only the register located at _address
	opcode = ADS1299SendSPI(0x00);
	StopWatch_DelayUs(T_SPI); // After the serial communication is finished, always wait four or more tCLK cycles before taking CS high

	ADS1299PinHigh(ADS1299_CS_PIN); //assert HIGH the chip select pin of ADS //close SPI
	if(is_running)
		ADS1299StartStreaming();
	return opcode;
}

void ADS1299PinHigh(pin_ADS1299_t pin)
{
	GPIOOn(pin);
}

void ADS1299PinLow(pin_ADS1299_t pin)
{
	GPIOOff(pin);
}

void ADS1299CS(uint8_t state)
{
	GPIOState(ADS1299_CS_PIN, state);
}

uint8_t ADS1299SendSPI(uint8_t tx)
{
	uint8_t rx;
	SpiReadWrite(SPI_1, &tx, &rx, 1);
	return rx;
//	return SpiFastTransfer(SPI_1, tx); // en teoria anda mejor con esta funcion. Queda implementado para prueba
}

void ADS1299SetChannelsToDefaultConfigForECG(void)
{
	CHnSet_ADS1299_t setting[8];
	uint8_t i;
	for(i=0 ; i<8 ; i++)
	{
		setting[i].power_down = true;
		setting[i].use_srb2 = false;
		setting[i].pga_gain = ADS1299_GAIN12;
		setting[i].channel_input = ADS1299_NORMAL;
	}
	setting[ADS1299_CHANNEL1].power_down = false;

	ADS1299ConfigAllChannels(setting);
	ADS1299WriteRegister(ADS1299_CONFIG1, 0x90 | ADS1299_SAMPLE_RATE_250HZ);
	ADS1299WriteRegister(ADS1299_CONFIG3,0xE0); // If Using Internal Reference, Send This Command WREG CONFIG3 E0h
//	ADS1299WriteRegister(ADS1299_MISC1,0x20);  // close SRB1 switch on-board
	ADS1299ConfigBiasDrive();
}

void ADS1299SetChannelsToDefaultConfigForEEG(void)
{
	CHnSet_ADS1299_t setting[8];
	uint8_t i;
	for(i=0 ; i<8 ; i++)
	{
		setting[i].power_down = true;
		setting[i].use_srb2 = false;
		setting[i].pga_gain = ADS1299_GAIN01;
		setting[i].channel_input = ADS1299_NORMAL;
	}
	setting[ADS1299_CHANNEL1].power_down = false;
	setting[ADS1299_CHANNEL2].power_down = false;
	setting[ADS1299_CHANNEL3].power_down = false;
	setting[ADS1299_CHANNEL4].power_down = false;
	setting[ADS1299_CHANNEL5].power_down = false;
	ADS1299ConfigAllChannels(setting);
	ADS1299WriteRegister(ADS1299_CONFIG1, 0x90 | ADS1299_SAMPLE_RATE_250HZ);
	ADS1299WriteRegister(ADS1299_CONFIG3,0xE0); // If Using Internal Reference, Send This Command WREG CONFIG3 E0h
	ADS1299WriteRegister(ADS1299_MISC1,0x20);  // close SRB1 switch on-board
	ADS1299ConfigBiasDrive();
}

void ADS1299SetChannelsToDefaultConfigForEMG(void)
{
	CHnSet_ADS1299_t setting[8];
	uint8_t i;
	for(i=0 ; i<8 ; i++)
	{
		setting[i].power_down = true;
		setting[i].use_srb2 = false;
		setting[i].pga_gain = ADS1299_GAIN12;    // Se puede de acá cambiar la ganancia para todos los canales
		setting[i].channel_input = ADS1299_NORMAL;
	}
	setting[ADS1299_CHANNEL1].power_down = false;
	setting[ADS1299_CHANNEL2].power_down = false;
	setting[ADS1299_CHANNEL3].power_down = false;
	setting[ADS1299_CHANNEL4].power_down = false;   // Acá habilito los canales que quiero muestrear
	setting[ADS1299_CHANNEL5].power_down = false;
	setting[ADS1299_CHANNEL6].power_down = false;
	//setting[ADS1299_CHANNEL7].power_down = false;
	//setting[ADS1299_CHANNEL8].power_down = false;

	ADS1299ConfigAllChannels(setting);
	ADS1299WriteRegister(ADS1299_CONFIG1, 0x90 | ADS1299_SAMPLE_RATE_2kHZ);  // Acá puedo cambiar la frecuencia de muestreo ADS1299_SAMPLE_RATE_2kHZ
	ADS1299WriteRegister(ADS1299_CONFIG3,0xE0); // If Using Internal Reference, Send This Command WREG CONFIG3 E0h
	//ADS1299WriteRegister(ADS1299_MISC1,0x20);  // close SRB1 switch on-board
	ADS1299ConfigBiasDrive();
}
void ADS1299ConfigOneChannel(channel_ADS1299_t channel, CHnSet_ADS1299_t config)
{
	ADS1299WriteRegister(ADS1299_CH1SET+channel, config.channel_input|config.pga_gain|(config.power_down<<7)|(config.use_srb2<<3));
	StopWatch_DelayMs(1);
	if(!config.power_down)
		channel_activ |= (1<<channel);
}

void ADS1299ConfigAllChannels(CHnSet_ADS1299_t *config)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ConfigOneChannel(ADS1299_CHANNEL1+i, config[i]);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ConfigOneChannel(ADS1299_CHANNEL1+i, config[i]);
}

void ADS1299ConfigAllChannelsEqual(CHnSet_ADS1299_t config)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ConfigOneChannel(ADS1299_CHANNEL1+i, config);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ConfigOneChannel(ADS1299_CHANNEL1+i, config);
}

void ADS1299ChangeSamplingFrecuency(frec_ADS1299_t frec)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CONFIG1);
	ADS1299WriteRegister(ADS1299_CONFIG1, (setting & 0xF8) | frec); //Turn off clk output if no slave present and always write 1 in bit 7 and 2 in bits 3 and 4
}

void ADS1299ChangeChannelInputType(channel_ADS1299_t channel, in_cod_ADS1299_t input_code)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CH1SET+channel);
	ADS1299WriteRegister(ADS1299_CH1SET+channel, ((setting & 0xF8) | input_code));
}

void ADS1299ChangeAllChannelInputType(in_cod_ADS1299_t *input_code)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelInputType(ADS1299_CHANNEL1+i, input_code[i]);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelInputType(ADS1299_CHANNEL1+i, input_code[i]);
}

void ADS1299ChangeAllChannelInputTypeEqual(in_cod_ADS1299_t input_code)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelInputType(ADS1299_CHANNEL1+i, input_code);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelInputType(ADS1299_CHANNEL1+i, input_code);
}

void ADS1299ChangeChannelPGAGain(channel_ADS1299_t channel, gain_ADS1299_t gain)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CH1SET+channel);
	ADS1299WriteRegister(ADS1299_CH1SET+channel, (setting & 0x8F) | gain); //Turn off clk output if no slave present and always write 1 in bit 7 and 2 in bits 3 and 4
}

void ADS1299ChangeAllChannelPGAGain(gain_ADS1299_t *gain)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1+i, gain[i]);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1+i, gain[i]);
}

void ADS1299ChangeAllChannelPGAGainEqual(gain_ADS1299_t gain)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1+i, gain);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ChangeChannelPGAGain(ADS1299_CHANNEL1+i, gain);
}

void ADS1299ActivateChannel(channel_ADS1299_t channel)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CH1SET+channel);
	ADS1299WriteRegister(ADS1299_CH1SET+channel, setting & 0x7F);
	channel_activ |= (1<<channel);
}

void ADS1299DeactivateChannel(channel_ADS1299_t channel)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CH1SET+channel);
	ADS1299WriteRegister(ADS1299_CH1SET+channel, setting | 0x80);
	channel_activ &= (0xFF^(1<<channel));
}

void ADS1299ActivateAllChannels(void)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299ActivateChannel(ADS1299_CHANNEL1+i);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299ActivateChannel(ADS1299_CHANNEL1+i);
}

void ADS1299DeactivateAllChannels(void)
{
	uint8_t i;
	if(is_running)
	{
		ADS1299StopStreaming();
		for(i=0 ; i<8 ; i++)
			ADS1299DeactivateChannel(ADS1299_CHANNEL1+i);
		ADS1299StartStreaming();
	}
	else
		for(i=0 ; i<8 ; i++)
			ADS1299DeactivateChannel(ADS1299_CHANNEL1+i);
}

void ADS1299ConfigBiasDrive(void)
{
	uint8_t setting;
	setting = ADS1299ReadRegister(ADS1299_CONFIG3);
	ADS1299WriteRegister(ADS1299_CONFIG3, setting | 0x0C);
	ADS1299WriteRegister(ADS1299_BIAS_SENSP,channel_activ);
	ADS1299WriteRegister(ADS1299_BIAS_SENSP,channel_activ);
}

bool ADS1299IsDataReady(void)
{
	return (!GPIORead(ADS1299_DRDY_PIN));
}

void ADS1299OneShotConversion(void)
{
	ADS1299PinLow(ADS1299_CS_PIN); //open SPI
	ADS1299SendCommand(ADS1299_RDATA);

	uint8_t i,j = 0; //loop counters

	for(j=0; j<3; j++) // read in the new channel data
		state[j] = ADS1299SendSPI(0x00);//read status register (1100 + LOFF_STATP + LOFF_STATN + GPIO[7:4])
	for(i=0; i<8; i++)
	{
		for(j=0; j<3; j++)
		{
			buffer[(i*3)+j] = ADS1299SendSPI(0x00); // Store data RAW
		}
	}

	ADS1299PinHigh(ADS1299_CS_PIN); //assert HIGH the chip select pin of ADS //close SPI
}

void ASD1299ConvertDataTo2ndComplement(int32_t *vector)
{
	uint8_t i = 0; //loop counters
	for(i=0; i<8; i++)
	{
		vector[i] = (buffer[i*3]<<16) | (buffer[(i*3)+1] << 8)| buffer[(i*3)+2]; //Store 24bits data in 32bits data vector
		if((((vector[i]) >> (23)) & 0x01) == 1)  // convert 3 byte 2's compliment to 4 byte 2's compliment
			vector[i] |= 0xFF000000;
	}
}

void ADS1299ConfigTestSignal(test_type_ADS1299_t type, test_amp_ADS1299_t amp)
{
	ADS1299WriteRegister(ADS1299_CONFIG2, 0xD0 | type | amp);
}

uint8_t ADS1299LeadOffDetection(uint8_t mode)
{
	ADS1299WriteRegister(ADS1299_LOFF, 0x02);
	ADS1299WriteRegister(ADS1299_CONFIG4, 0x02);
	return 0;
}

void ADS1299SetGPIO(uint8_t gpio)
{
	ADS1299WriteRegister(ADS1299_GPIO, gpio<<4);
}


uint8_t ADS1299GetGPIO(void)
{
	return (ADS1299ReadRegister(ADS1299_GPIO) & (0xF0>>4));
}
