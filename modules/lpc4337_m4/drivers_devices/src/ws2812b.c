/**
 * @file WS2812b.c
 * @author  Franco Cipriani
 * @date 10/11/2018
 * @version 1.0
 */

/*==================[inclusions]=============================================*/
#include "ws2812b.h"

/*==================[variables]==============================================*/
const uint8_t mask_hex[] = {128, 64, 32, 16, 8, 4, 2, 1};

//const uint8_t mask_hex[] = {1, 2, 4, 8,16, 32, 64, 128};

uint8_t pin_number;

//Conjunto vectorizado de Estructuras Para inicializar los GPIO
const digitalIO LEDs [] =
{
		//Puerto, Pin ,gpioPort ,gpioPin ,Modo
		//{0x06, 0x01, 0x03, 0x00,  SCU_MODE_INACT | SCU_MODE_FUNC0 | SCU_MODE_HIGHSPEEDSLEW_EN | SCU_MODE_ZIF_DIS}, //GPIO0
		{0x06, 0x01, 0x03, 0x00,  SCU_PINIO_FAST | SCU_MODE_FUNC0},
		{0x06, 0x04, 0x03, 0x03,  SCU_PINIO_FAST | SCU_MODE_FUNC0}, //GPIO1
		{0x06, 0x05, 0x03, 0x04,  SCU_PINIO_FAST | SCU_MODE_FUNC0}, //GPIO2
		{0x06, 0x07, 0x05, 0x0F,  SCU_PINIO_FAST | SCU_MODE_FUNC4}, //GPIO3
		{0x06, 0x08, 0x05, 0x10,  SCU_PINIO_FAST | SCU_MODE_FUNC4}, //GPIO4
		{0x06, 0x09, 0x03, 0x05,  SCU_PINIO_FAST | SCU_MODE_FUNC0}, //GPIO5
		{0x06, 0x0A, 0x03, 0x06,  SCU_PINIO_FAST | SCU_MODE_FUNC0}, //GPIO6
		{0x06, 0x0B, 0x03, 0x07,  SCU_PINIO_FAST | SCU_MODE_FUNC0}, //GPIO7
		{0x06, 0x0C, 0x02, 0x08,  SCU_PINIO_FAST | SCU_MODE_FUNC0}  //GPIO8
};

// Save Interrupt
uint8_t save_int[53] = {0}; // 0 (Disable) / 1 (Active)
/*==================[functions]==============================================*/
void WS2812bInit(uint8_t pin) // GPIO0 - GPIO8
{
	Ws2812bConfigGPIO(pin);
	pin_number = pin;
}

void Ws2812bSend(bit_send data)
{
	Ws2812bInterruptDisable();

	uint8_t i;

	// Blue
	for( i = 0; i <= 7 ; i++)
		{
			if(mask_hex[i]  == (data.G & mask_hex[i]))
			{
					Ws2812bSendHigh(pin_number);
			}
			else
			{
					Ws2812bSendLow(pin_number);
			}
		}
	// Red
		for( i = 0; i <= 7 ; i++)
		{
			if(mask_hex[i]  == (data.R & mask_hex[i]))
			{
				Ws2812bSendHigh(pin_number);
			}
			else
			{
				Ws2812bSendLow(pin_number);
			}
		}
	// Gren
		for( i = 0; i <= 7 ; i++)
		{
			if(mask_hex[i] == (data.B & mask_hex[i]))
			{
				Ws2812bSendHigh(pin_number);
			}
			else
			{
				Ws2812bSendLow(pin_number);
			}
		}
	Ws2812bInterruptActive();
}

inline void Ws2812bSendHigh(uint8_t pin) // Mandar uno
{
	Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
	nada();
	nada();
	nada();
	nada();
	nada();
	nada();
	nada();
	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
	nada();

}

inline void Ws2812bSendLow(uint8_t pin) // Mandar cero
{
	Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
	nada();
	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
	nada();
	nada();
	nada();
	nada();
	nada();
	nada();
	nada();
}

void Ws2812bSendRet(void)
{
	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin_number].GpioPort , LEDs[pin_number].GpioPin );
	DelayUs(RET);
}

void  Ws2812bConfigGPIO(uint8_t pin)
{
	Chip_GPIO_Init( LPC_GPIO_PORT );
	Chip_SCU_PinMuxSet( LEDs[pin].hwPort , LEDs[pin].hwPin , LEDs[pin].modo );
	Chip_GPIO_SetPinDIROutput( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
	Chip_GPIO_SetPortOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );
}

void nada(void){} // the magic function - 100ns delay

void Ws2812bTest(void)
{
	while(1)
	{
		Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[0].GpioPort , LEDs[0].GpioPin );
		Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[0].GpioPort , LEDs[0].GpioPin );
	}
}


void Ws2812bInterruptDisable(void)
{
	__disable_irq();
}

void Ws2812bInterruptActive(void)
{
	__enable_irq();
}

















/*
			                            NO PROBADO A FALTA DE OSCILOSCOPIO
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");

	//StopWatch_DelayTicks(T1H);
	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");

	//StopWatch_DelayTicks(T1L);
}

inline void Ws2812bSendLow(uint8_t pin) // Mandar cero
{
	Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");


	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");
	__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");__asm volatile("nop");

}



*
*
*
*
*
*inline void Ws2812bSendHigh(uint8_t pin) // Mandar uno
{
	Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");

	//StopWatch_DelayTicks(T1H);
	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");

	//StopWatch_DelayTicks(T1L);
}

inline void Ws2812bSendLow(uint8_t pin) // Mandar cero
{
	Chip_GPIO_SetPinOutHigh( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );

	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");

	Chip_GPIO_SetPinOutLow( LPC_GPIO_PORT, LEDs[pin].GpioPort , LEDs[pin].GpioPin );


	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");
	__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");__asm ("nop");

}
 */
