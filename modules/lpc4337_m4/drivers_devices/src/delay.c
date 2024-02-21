/* @brief  EDU-CIAA NXP GPIO driver
 * @author Albano Peñalva
 *
 * This driver provide functions to generate delays using Timer0
 * of LPC4337
 *
 * @note
 *
 * @section changelog
 *
 * |   Date	    | Description                                    |
 * |:----------:|:-----------------------------------------------|
 * | 26/11/2018 | Document creation		                         |
 *
 */

#include "delay.h"
#include "chip.h"

/*****************************************************************************
 * Private macros/types/enumerations/variables definitions
 ****************************************************************************/

#define TMR_RST 0x02			/*!< Reset timer */
#define PR_ZERO 0x00			/*!< Prescaler zero */
#define STOP_ON_MATCH 0x04		/*!< Stop timer on match */
#define TMR_START 0x01			/*!< Start timer */
#define RST_INT_ALL 0xFF		/*!< Reset all interrupts */
#define MSK_TMR_ON 0x01			/*!< Timer start mask */

/*****************************************************************************
 * Public types/enumerations/variables declarations
 ****************************************************************************/

/*****************************************************************************
 * Private functions definitions
 ****************************************************************************/

/*****************************************************************************
 * Private functions declarations
 ****************************************************************************/

/*****************************************************************************
 * Public functions declarations
 ****************************************************************************/

void DelaySec(uint32_t sec)
{
	DelayMs(sec * 1000);
}

void DelayMs(uint32_t msec)
{
	LPC_TIMER0->TCR = TMR_RST;
	LPC_TIMER0->PR = 0x00;
	LPC_TIMER0->MR[0] = msec * ((SystemCoreClock) / 1000 - 1);
	LPC_TIMER0->IR = 0xff;
	LPC_TIMER0->MCR = STOP_ON_MATCH;
	LPC_TIMER0->TCR = TMR_START;

	/* Wait until delay time has elapsed */
	while (LPC_TIMER0->TCR & MSK_TMR_ON);
}

void DelayUs(uint32_t usec)
{
	LPC_TIMER0->TCR = TMR_RST;
	LPC_TIMER0->PR = 0x00;
	LPC_TIMER0->MR[0] = usec * ((SystemCoreClock) / 1000000 - 1);
	LPC_TIMER0->IR = 0xff;
	LPC_TIMER0->MCR = STOP_ON_MATCH;
	LPC_TIMER0->TCR = TMR_START;

	/* Wait until delay time has elapsed */
	while (LPC_TIMER0->TCR & MSK_TMR_ON);
}
