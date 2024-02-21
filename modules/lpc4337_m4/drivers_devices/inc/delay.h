/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup Delay
 ** @{ */

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

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>

/*****************************************************************************
 * Public macros/types/enumerations/variables definitions
 ****************************************************************************/

/*****************************************************************************
 * Public functions definitions
 ****************************************************************************/

/**
 * @brief Delay in seconds
 * @param[in] sec seconds to be in delay
 * @return None
 */
void DelaySec(uint32_t sec);

/**
 * @brief Delay in milliseconds
 * @param[in] msec milliseconds to be in delay
 * @return None
 */
void DelayMs(uint32_t msec);

/**
 * @brief Delay in microseconds
 * @param[in] usec microseconds to be in delay
 * @return None
 */
void DelayUs(uint32_t usec);
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
#endif /* DELAY_H_ */
