/* Copyright 2016, 
 * Leandro D. Medus
 * lmedus@bioingenieria.edu.ar
 * Eduardo Filomena
 * efilomena@bioingenieria.edu.ar
 * Juan Manuel Reta
 * jmrera@bioingenieria.edu.ar
 * Esteban Osella
 * esteban.osella@uner.edu.ar
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

/*
 * Initials     Name
 * ---------------------------
 *	LM			Leandro Medus
 *  EF			Eduardo Filomena
 *  JMR			Juan Manuel Reta
 *  OE			Esteban Osella
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160422 v0.1 initials initial version leo
 * 20160807 v0.2 modifications and improvements made by Eduardo Filomena
 * 20160808 v0.3 modifications and improvements made by Juan Manuel Reta
 * 20231005 v0.4 deletion of the typdefs usage regarding USB_UART, RS232_UART, RS485_UART
 * 				 and using LPC_USARTx instead, by EO
 */

/*==================[inclusions]=============================================*/
#include "UART.h"

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

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief UART Initialization method  */
uint32_t Init_Uart_Ftdi(int32_t baudios) {
	uint32_t ret = 0;
	Chip_UART_Init(LPC_USART2);
	ret = Chip_UART_SetBaudFDR(LPC_USART2, baudios);
	Chip_UART_ConfigData( LPC_USART2,
		UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);
	Chip_UART_SetupFIFOS(LPC_USART2, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

	Chip_UART_TXEnable(LPC_USART2);

	Chip_SCU_PinMux(UART_USB_TXD_MUX_GROUP, UART_USB_TXD_MUX_PIN, MD_PDN,
	FUNC6); /* P7_1: UART2_TXD */
	Chip_SCU_PinMux(UART_USB_RXD_MUX_GROUP, UART_USB_RXD_MUX_PIN,
	MD_PLN | MD_EZI | MD_ZI, FUNC6); /* P7_2: UART2_RXD */

	return ret;
}

/** \brief UART Initialization method  */
uint32_t Init_Uart_Ftdi_IntAct(int32_t baudios) {
	Chip_UART_Init(LPC_USART2);
	Chip_UART_ConfigData( LPC_USART2,
	UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS);

	Chip_UART_SetBaudFDR(LPC_USART2, baudios);

	Chip_UART_SetupFIFOS(LPC_USART2,
			(UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS
					| UART_FCR_TRG_LEV3));

	Chip_UART_IntEnable( LPC_USART2, ( UART_IER_RBRINT | UART_IER_RLSINT));

	Chip_SCU_PinMux(UART_USB_TXD_MUX_GROUP, UART_USB_TXD_MUX_PIN, MD_PDN,
			FUNC6); /* P7_1: UART2_TXD */
	Chip_SCU_PinMux(UART_USB_RXD_MUX_GROUP, UART_USB_RXD_MUX_PIN,
	SCU_MODE_INACT | SCU_MODE_INBUFF_EN | MD_PLN | MD_EZI | MD_ZI, FUNC6); /* P7_2: UART2_RXD */

	return TRUE;
}

uint32_t Init_Uart_Rs485(void) {

	/* UART0 (RS485/Profibus) */
	Chip_UART_Init(LPC_USART0);
	Chip_UART_SetBaudFDR(LPC_USART0, 921600);
 	Chip_UART_SetupFIFOS(LPC_USART0, UART_FCR_FIFO_EN | UART_FCR_TX_RS | UART_FCR_RX_RS | UART_FCR_TRG_LEV3); // Estaba en UART_FCR_TRG_LEV0
 	// Esto ultimo es que tan lleno tiene que estar el buffer para iniciar el envio

 	Chip_UART_ReadByte(LPC_USART0);
	Chip_UART_TXEnable(LPC_USART0);//RS485_UART

	Chip_SCU_PinMux(RS485_TXD_MUX_GROUP, RS485_TXD_MUX_PIN, MD_PDN, FUNC7); /* P9_5: UART0_TXD */
	Chip_SCU_PinMux(RS485_RXD_MUX_GROUP, RS485_RXD_MUX_PIN,	MD_PLN | MD_EZI | MD_ZI, FUNC7); /* P9_6: UART0_RXD */

	Chip_UART_SetRS485Flags(LPC_USART0,	UART_RS485CTRL_DCTRL_EN | UART_RS485CTRL_OINV_1);
	Chip_SCU_PinMux(6, 2, MD_PDN, FUNC2); /* P6_2: UART0_DIR */

	return TRUE;
}

uint32_t Init_Uart_Rs232(void) {

	/*UART initialization*/

	/* UART3 (RS232) */
	Chip_UART_Init(LPC_USART3);
	Chip_UART_SetBaudFDR(LPC_USART3, 921600);

	Chip_UART_SetupFIFOS(LPC_USART3, UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0);

	Chip_UART_TXEnable(LPC_USART3);

	Chip_SCU_PinMux(RS232_TXD_MUX_GROUP, RS232_TXD_MUX_PIN, MD_PDN, FUNC2); /* P2_3: UART3_TXD */
	Chip_SCU_PinMux(RS232_TXD_MUX_GROUP, RS232_RXD_MUX_PIN,
	MD_PLN | MD_EZI | MD_ZI, FUNC2); /* P2_4: UART3_RXD */

	return TRUE;
}

uint32_t ReadStatus_Uart_Rs232(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART3) & UART_LSR_THRE;
}

uint32_t ReadStatus_Uart_Ftdi(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART2) & UART_LSR_THRE;

}

uint32_t ReadStatus_Uart_Rs485(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART0) & UART_LSR_THRE;

}

uint32_t ReadRxReady_Uart_Ftdi(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART2) & UART_LSR_RDR;
}

uint32_t ReadRxReady_Uart_Rs232(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART3) & UART_LSR_RDR;
}

uint32_t ReadRxReady_Uart_RS485(void) {
	return Chip_UART_ReadLineStatus((LPC_USART_T *) LPC_USART0) & UART_LSR_RDR;
}

uint8_t ReadByte_Uart_Ftdi(uint8_t* dat) {
	if (ReadRxReady_Uart_Ftdi()) {
		*dat = Chip_UART_ReadByte((LPC_USART_T *) LPC_USART2);
		return TRUE;
	} else {
		return FALSE;
	}
}

uint8_t ReadByte_Uart_Rs232(uint8_t* dat) {
	if (ReadRxReady_Uart_Rs232()) {
		*dat = Chip_UART_ReadByte((LPC_USART_T *) LPC_USART3);
		return TRUE;
	} else {
		return FALSE;
	}
}

uint8_t ReadByte_Uart_RS485(uint8_t* dat) {
	if (ReadRxReady_Uart_RS485()) {
		*dat = Chip_UART_ReadByte((LPC_USART_T *) LPC_USART0);
		return TRUE;
	} else {
		return FALSE;
	}
}



void Send_Byte_UART(uint8_t* dat) {
	Chip_UART_SendByte(LPC_USART2, (uint8_t) * dat);
}

void SendByte_Uart_Rs232(uint8_t* dat) {
	/* sending byte */

	while (ReadStatus_Uart_Rs232() == 0)
		;
	Chip_UART_SendByte((LPC_USART_T *) LPC_USART2, (uint8_t) * dat);

}

void Send_Byte_RS485(uint8_t* dat) {
	Chip_UART_SendByte(LPC_USART0, (uint8_t) * dat);
}

void SendString_Uart_Ftdi(uint8_t* msg) {
	/* sending byte by byte*/
	while (*msg != 0) {
		while (ReadStatus_Uart_Ftdi() == 0)
			;
		Chip_UART_SendByte((LPC_USART_T *) LPC_USART2, (uint8_t) * msg);
		msg++;
	}
}

void SendString_Uart_Rs232(uint8_t* msg) {
	/* sending byte by byte*/
	while (*msg != 0) {
		while (ReadStatus_Uart_Rs232() == 0)
			;
		Chip_UART_SendByte((LPC_USART_T *) LPC_USART2, (uint8_t) * msg);
		msg++;
	}
}

void Send_String_UART(const void *data, int numBytes) {
	Chip_UART_SendBlocking(LPC_USART2, data, numBytes);
}

void SendStringRs485(const void *data, int numBytes) {
	Chip_UART_SendBlocking(LPC_USART0, data, numBytes);
}

void IntToString(int16_t value, uint8_t* pBuf, uint32_t len, uint32_t base) {
	/**
	 * \details
	 * Conversion method to obtain a character or a string from a float to send
	 * throw UART peripheral.
	 * */
	static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
	int pos = 0;
	int tmpValue = value;

	/*  the buffer must not be null and at least have a length of 2 to handle one */
	/*  digit and null-terminator */
	if (pBuf == NULL || len < 2) {
		return;
	}

	/* a valid base cannot be less than 2 or larger than 36 */
	/* a base value of 2 means binary representation. A value of 1 would mean only zeros */
	/*  a base larger than 36 can only be used if a larger alphabet were used. */
	if (base < 2 || base > 36) {
		return;
	}

	/* negative value */
	if (value < 0) {
		tmpValue = -tmpValue;
		value = -value;
		pBuf[pos++] = '-';
	}

	/* calculate the required length of the buffer */
	do {
		pos++;
		tmpValue /= base;
	} while (tmpValue > 0);

	if (pos > len) {
		/* the len parameter is invalid. */
		return;
	}

	pBuf[pos] = '\0';

	do {
		pBuf[--pos] = pAscii[value % base];
		value /= base;
	} while (value > 0);

	return;
}




/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base) {
	// check that the base if valid
	if (base < 2 || base > 36) {
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do {
		tmp_value = value;
		value /= base;
		*ptr++ =
				"zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
						+ (tmp_value - value * base)];
	} while (value);

	// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr) {
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
