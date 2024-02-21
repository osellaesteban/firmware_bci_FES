/* Copyright 2016, 
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
 * Sebastián Mateos
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

/** \brief Bare Metal driver for uart in the EDU-CIAA board.
 **
 **/

/*==================[inclusions]=============================================*/
#include "uart.h"

/*==================[macros and definitions]=================================*/

#define DELAY_CHARACTER 500000

/* UART0 (RS485/Profibus) */

#define RS485_TXD_MUX_GROUP   9
#define RS485_RXD_MUX_GROUP   9

#define RS485_TXD_MUX_PIN   5
#define RS485_RXD_MUX_PIN   6


/* UART2 (USB-UART) */
#define UART_USB_TXD_MUX_GROUP   7
#define UART_USB_RXD_MUX_GROUP   7

#define UART_USB_TXD_MUX_PIN   1
#define UART_USB_RXD_MUX_PIN   2


/* UART3 (RS232) */

#define RS232_TXD_MUX_GROUP   2
#define RS232_RXD_MUX_GROUP   2

#define RS232_TXD_MUX_PIN   3
#define RS232_RXD_MUX_PIN   4


/*Direction Pin*/
#define DIR_RS485_MUX_GROUP 	6
#define DIR_RS485_MUX_PIN 	2

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

void (*serial_pc)()= NO_INT;
void (*serial_p2_connector)()= NO_INT;
void (*serial_rs485)()= NO_INT;
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief ADC Initialization method  */
uint32_t UartInit(serial_config *port)
{
	switch(port->port)
	{
	 	case SERIAL_PORT_PC:

	 		Chip_UART_Init(LPC_USART2);
	 		Chip_UART_SetBaudFDR(LPC_USART2, port->baud_rate);
	 		Chip_UART_ConfigData(LPC_USART2, UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	 		Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);
	 		Chip_UART_TXEnable(LPC_USART2);
	 		Chip_SCU_PinMux(UART_USB_TXD_MUX_GROUP, UART_USB_TXD_MUX_PIN, MD_PDN, FUNC6);              /* P7_1: UART2_TXD */
	 		Chip_SCU_PinMux(UART_USB_RXD_MUX_GROUP, UART_USB_RXD_MUX_PIN, MD_PLN|MD_EZI|MD_ZI, FUNC6); /* P7_2: UART2_RXD */

	 		serial_pc=port->pSerial;

	 		if(serial_pc != NO_INT)
	 		{
	 			Chip_UART_IntEnable(LPC_USART2, UART_IER_RBRINT);
	 			NVIC_EnableIRQ(USART2_IRQn);
	 		}

	 	break;

	 	case SERIAL_PORT_P2_CONNECTOR:

			Chip_UART_Init(LPC_USART3);
			Chip_UART_SetBaudFDR(LPC_USART3, port->baud_rate);

			Chip_UART_SetupFIFOS(LPC_USART3, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

			Chip_UART_TXEnable(LPC_USART3);

			Chip_SCU_PinMux(RS232_TXD_MUX_GROUP, RS232_TXD_MUX_PIN, MD_PDN, FUNC2);              /* P2_3: UART3_TXD */
			Chip_SCU_PinMux(RS232_TXD_MUX_GROUP, RS232_RXD_MUX_PIN, MD_PLN|MD_EZI|MD_ZI, FUNC2); /* P2_4: UART3_RXD */

			serial_p2_connector=port->pSerial;

	 		if(serial_p2_connector!= NO_INT)
	 		{
	 			Chip_UART_IntEnable(LPC_USART3, UART_IER_RBRINT);
	 			NVIC_EnableIRQ(USART3_IRQn);
	 		}


	 	break;

	 	case SERIAL_PORT_RS485:
	 		Chip_UART_Init(LPC_USART0);
	 		Chip_UART_SetBaudFDR(LPC_USART0, port->baud_rate);

	 		Chip_UART_SetupFIFOS(LPC_USART0, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

	 		Chip_UART_TXEnable(LPC_USART0);

	 		Chip_SCU_PinMux(RS485_TXD_MUX_GROUP, RS485_TXD_MUX_PIN, MD_PDN, FUNC7);              /* P9_5: UART0_TXD */
	 		Chip_SCU_PinMux(RS485_RXD_MUX_GROUP, RS485_RXD_MUX_PIN, MD_PLN|MD_EZI|MD_ZI, FUNC7); /* P9_6: UART0_RXD */

	 		Chip_UART_SetRS485Flags(LPC_USART0, UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1);

	 		Chip_SCU_PinMux(DIR_RS485_MUX_GROUP, DIR_RS485_MUX_PIN, MD_PDN, FUNC2);              /* P6_2: UART0_DIR */

	 		serial_rs485=port->pSerial;

	 		if(serial_rs485 != NO_INT)
	 		{

	 			Chip_UART_IntEnable(LPC_USART0, UART_IER_RBRINT);
		 		NVIC_EnableIRQ(USART0_IRQn);
	 		}


	 	break;
	}
return TRUE;
}

uint32_t UartReadStatus(uint8_t port)
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	return Chip_UART_ReadLineStatus(pUsart) & UART_LSR_THRE;

}
uint32_t UartRxReady(uint8_t port)
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	return Chip_UART_ReadLineStatus(pUsart) & UART_LSR_RDR;
}

uint8_t UartReadByte(uint8_t port, uint8_t* dat)
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	if(UartRxReady(port))
	{
		*dat = Chip_UART_ReadByte(pUsart);
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void UartSendByte(uint8_t port,uint8_t* dat)
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	while(UartReadStatus(port) == 0);
	Chip_UART_SendByte(pUsart, *dat);

}
void UartSendString(uint8_t port, uint8_t *msg)/*Stops on '\0'*/
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	while(*msg != 0)
	{
		while( UartReadStatus(port) == 0);
		Chip_UART_SendByte(pUsart, (uint8_t)*msg);
		msg++;
	}
}
void UartSendBuffer(uint8_t port, const void *data, uint8_t nbytes)  /*Send n Bytes - Inline candidate...*/
{
	LPC_USART_T *pUsart;
	switch(port)
	{
	case SERIAL_PORT_PC:
		pUsart = LPC_USART2;
	break;
	case SERIAL_PORT_P2_CONNECTOR:
		pUsart = LPC_USART3;
	break;
	case SERIAL_PORT_RS485:
		pUsart = LPC_USART0;
	break;
	}

	Chip_UART_SendBlocking(pUsart,data, nbytes);
}

uint8_t* UartItoa(uint32_t val, uint8_t base)
{
	static char buf[32] = {0};

	uint32_t i = 30;

	for(; val && i ; --i, val /= base)

		buf[i] = "0123456789abcdef"[val % base];

	return &buf[i+1];
}
/*
void UART0_IRQHandler(void)
{
	if(serial_rs485 != NO_INT)
	{
		serial_rs485();
	}
}
*/
void UART2_IRQHandler(void)
{
	if(serial_pc != NO_INT)
	{
		serial_pc();
	}
}

void UART3_IRQHandler(void)
{
	if(serial_p2_connector != NO_INT)
	{
		serial_p2_connector();
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
