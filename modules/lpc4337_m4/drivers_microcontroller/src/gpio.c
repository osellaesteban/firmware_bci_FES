/* Copyright 2018,
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

/** \addtogroup GIOP
 ** @{

/** \brief Bare Metal driver for gpio in the EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 *  SM			Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190220 v0.1 SM initial version Sebastian Mateos
 * 20190820 v0.2 SM modifications and improvements Sebastian Mateos
 */

/*==================[inclusions]=============================================*/

#include "gpio.h"
#include "chip.h"
#include <stdint.h>
#include "bool.h"

/*==================[macros and definitions]=================================*/

/** @typedef digitalIO
 * @brief Opciones de configuracion para los gpio
 */
typedef struct
{
	uint8_t hwPort; /**< Puerto del hardware*/
	uint8_t hwPin; /**< Pin del hardware*/
	uint8_t gpioPort; /**< Puerto del gpio*/
	uint8_t gpioPin; /**< Pin del gpio*/
	uint16_t mode; /**< Opciones de configuracion del gpio*/
} digitalIO;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO0_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO1_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO2_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO3_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO4_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO5_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO6_IRQHandler(void);

/** @brief Interruption function of a gpio entry. Call the indicated function.
 */
void GPIO7_IRQHandler(void);

/** @brief Interruption function of a group of gpio entries. Call the indicated function.
 */
void GINT0_IRQHandler(void);

/*==================[internal data definition]===============================*/

const digitalIO gpio[] =
{
	{0x01, 0x00, 0x00, 0x04, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration TEC1*/
	{0x01, 0x01, 0x00, 0x08, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration TEC2*/
	{0x01, 0x02, 0x00, 0x09, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration TEC3*/
	{0x01, 0x06, 0x01, 0x09, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration TEC4*/
	{0x02, 0x0A, 0x00, 0x0E, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LED1*/
	{0x02, 0x0B, 0x01, 0x0B, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LED2*/
	{0x02, 0x0C, 0x01, 0x0C, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LED3*/
	{0x02, 0x00, 0x05, 0x00, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LEDRGB_R*/
	{0x02, 0x01, 0x05, 0x01, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LEDRGB_G*/
	{0x02, 0x02, 0x05, 0x02, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LEDRGB_B*/
	{0x04, 0x04, 0x02, 0x04, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LCD1*/
	{0x04, 0x05, 0x02, 0x05, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LCD2*/
	{0x04, 0x06, 0x02, 0x06, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration LCD3*/
	{0x04, 0x0A, 0x05, 0x0E, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LCD4*/
	{0x06, 0x04, 0x03, 0x03, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration GPIO1*/
	{0x06, 0x07, 0x05, 0x0F, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration GPIO3*/
	{0x06, 0x09, 0x03, 0x05, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration GPIO5*/
	{0x04, 0x09, 0x05, 0x0D, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LCD_EN*/
	{0x04, 0x08, 0x05, 0x0C, MD_PUP|MD_EZI|MD_ZI|FUNC4}, /* Configuration LCD_RS*/
	{0x01, 0x05, 0x01, 0x08, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration T_COL0*/
	{0x04, 0x00, 0x02, 0x00, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration T_FIL0*/
	{0x04, 0x02, 0x02, 0x02, MD_PUP|MD_EZI|MD_ZI|FUNC0}, /* Configuration T_FIL2*/
	{0x04, 0x03, 0x02, 0x03, MD_PUP|MD_EZI|MD_ZI|FUNC0}  /* Configuration T_FIL3*/

};

void (*ptr_GPIO_int_func[8])(); /**< Pointer to the function to be called at the interruption of each GPIO */
void (*ptr_GPIO_group_int_func[2])(); /**< Pointer to the function to be called at the interruption of a group of GPIOs */

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/


void GPIOInit(gpio_t pin, io_t io)
{
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Chip_SCU_PinMuxSet(gpio[pin].hwPort, gpio[pin].hwPin, gpio[pin].mode); /* Asocia el pin fisico del micro a un gpio */
	Chip_GPIO_SetPinDIR(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin, io); /* Indica si el gpio sera de salida o entrada */
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin, FALSE); /* Inicializa el gpio en 0 */
}

void GPIOOn(gpio_t pin)
{
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin);
}

void GPIOOff(gpio_t pin)
{
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin);
}

void GPIOState(gpio_t pin, bool state)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin, state);
}

void GPIOToggle(gpio_t pin)
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin);
}

bool GPIORead(gpio_t pin)
{
	return(Chip_GPIO_ReadPortBit(LPC_GPIO_PORT, gpio[pin].gpioPort, gpio[pin].gpioPin));
}

void GPIOActivInt(gpiogp_t gp, gpio_t pin, void *ptr_int_func, bool edge)
{
	ptr_GPIO_int_func[gp] = ptr_int_func;

	Chip_SCU_GPIOIntPinSel(gp, gpio[pin].gpioPort, gpio[pin].gpioPin); /* Configura el canal de la interrupcion*/

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Limpia el estado de la interrupcion*/
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Interrupcion por flanco*/
	if(edge)
		Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Interrupcion cuando el flanco es ascendente*/
	else
		Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(gp)); /* Interrupcion cuando el flanco es descendente*/

	NVIC_ClearPendingIRQ(32+gp); /* Limpia la interrupcion de PIN_INTX_IRQn definido en cmsis_43xx.h*/
	NVIC_EnableIRQ(32+gp); /* Habilita la interrupcion de PIN_INTX_IRQn definido en cmsis_43xx.h*/
}

void GPIOActivGroupInt(gpiogroupgp_t group_gp, gpio_t *pins, uint8_t n_pins, void *ptr_int_func, bool edge)
{
	uint8_t i;
	ptr_GPIO_group_int_func[group_gp] = ptr_int_func;
	Chip_GPIOGP_SelectOrMode(LPC_GPIOGROUP, group_gp); /* Cualquier GPIO dispara la interrupcion*/
	Chip_GPIOGP_SelectEdgeMode(LPC_GPIOGROUP, group_gp); /* Interrupcion por flanco*/
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, group_gp); /* Limpia el estado de la interrupcion*/
	for(i=1 ; i<n_pins ; i++)
	{
		if(edge)
			Chip_GPIOGP_SelectHighLevel(LPC_GPIOGROUP, group_gp, gpio[pins[i]].gpioPort, 1<<gpio[pins[i]].gpioPin);/* Interrupcion por nivel alto*/
		else
			Chip_GPIOGP_SelectLowLevel(LPC_GPIOGROUP, group_gp, gpio[pins[i]].gpioPort, 1<<gpio[pins[i]].gpioPin);/* Interrupcion por nivel bajo*/
		Chip_GPIOGP_EnableGroupPins(LPC_GPIOGROUP, group_gp, gpio[pins[i]].gpioPort, 1<<gpio[pins[i]].gpioPin); /* Habilita el pin para la interrupcion de grupo*/
	}
    NVIC_EnableIRQ(40+group_gp); /* Habilita la interrupcion de GINTn_IRQn definido en cmsis_43xx.h*/
}

void GPIO0_IRQHandler(void) //32
{
	GPIOCleanInt(GPIOGP0);
	ptr_GPIO_int_func[GPIOGP0]();
}

void GPIO1_IRQHandler(void) //33
{
	GPIOCleanInt(GPIOGP1);
	ptr_GPIO_int_func[GPIOGP1]();
}

void GPIO2_IRQHandler(void) //34
{
	GPIOCleanInt(GPIOGP2);
	ptr_GPIO_int_func[GPIOGP2]();
}

void GPIO3_IRQHandler(void) //35
{
	GPIOCleanInt(GPIOGP3);
	ptr_GPIO_int_func[GPIOGP3]();
}

void GPIO4_IRQHandler(void) //36
{
	GPIOCleanInt(GPIOGP4);
	ptr_GPIO_int_func[GPIOGP4]();
}

void GPIO5_IRQHandler(void) //37
{
	GPIOCleanInt(GPIOGP5);
	ptr_GPIO_int_func[GPIOGP5]();
}

void GPIO6_IRQHandler(void) //38
{
	GPIOCleanInt(GPIOGP6);
	ptr_GPIO_int_func[GPIOGP6]();
}

void GPIO7_IRQHandler(void) //39
{
	GPIOCleanInt(GPIOGP7);
	ptr_GPIO_int_func[GPIOGP7]();
}

void GINT0_IRQHandler(void) // 40
{
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, GPIOGROUPGP0);
	ptr_GPIO_group_int_func[GPIOGROUPGP0]();
}

void GINT1_IRQHandler(void) // 41
{
	Chip_GPIOGP_ClearIntStatus(LPC_GPIOGROUP, GPIOGROUPGP0);
	ptr_GPIO_group_int_func[GPIOGROUPGP1]();
}

void GPIODeinit(void)
{
	Chip_GPIO_DeInit(LPC_GPIO_PORT);
}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
