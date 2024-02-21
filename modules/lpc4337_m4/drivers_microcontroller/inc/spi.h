/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Microcontroller Drivers microcontroller
 ** @{ */
/** @addtogroup SPI
 ** @{ */

/** @file spi.h
 * @brief spi EDU-CIAA NXP SPI driver
 *
 * This driver provide functions to configure and handle the EDU-CIAA NXP SPI port
 * (named here SPI1) which use the SSP1 module of the LPC4337.
 *
 * @note This is not an SSP driver, therefore the other protocols supported for
 * this module are not implemented, just the SPI mode.
 *
 * @note This driver provide the functions to manage just the SSP1 module because
 * is the only one implemented in the EDU-CIAA NXP. However, it remains compatible
 * for future upgrades.
 *
 * @note This driver is limited to transfer data only in 8 bits format.
 *
 * @author Albano Peñalva
 *
 * @section changelog
 *
 * |   Date	    | Description                                    						|
 * |:----------:|:----------------------------------------------------------------------|
 * | 09/11/2018 | Document creation		                         						|
 * | 17/12/2018 | Added capability to transfer data in Polling, Interrupt and DMA modes |
 * | 20/12/2018 | Added capability to handle CS pin	 			 						|
 *
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

/*****************************************************************************
 * Public macros/types/enumerations/variables definitions
 ****************************************************************************/

/**
 * @brief SSP ports
 */
typedef enum {
	SPI_0,			/*!< SSP0 port (not implemented) */
	SPI_1			/*!< SSP1 port */
} spiPort_t;

/**
 * @brief SPI operating modes
 */
typedef enum {
	SLAVE = 0,		/*!< SPI slave mode */
	MASTER = 1		/*!< SPI master mode */
} spiMode_t;

/**
 * @brief SPI phase and polarity modes
 */
typedef enum {
	MODE0,		/*!< phase = 0 and polarity = 0 */
	MODE1,		/*!< phase = 1 and polarity = 0 */
	MODE2,		/*!< phase = 0 and polarity = 1 */
	MODE3		/*!< phase = 1 and polarity = 1 */
} clkMode_t;

/**
 * @brief SSP transfer modes
 */
typedef enum {
	SPI_POLLING,		/*!< Polling mode */
	SPI_INTERRUPT,		/*!< Interrupt mode */
	SPI_DMA				/*!< DMA mode */
} transferMode_t;

/**
 * @brief SPI module configuration structure
 */
typedef struct
{
	spiPort_t port;					/*!< Port number */
	spiMode_t mode;					/*!< Operating modes */
	clkMode_t clk_mode;				/*!< Phase and Polarity modes */
	uint32_t bitrate;				/*!< Transfer speed */
	transferMode_t transfer_mode;	/*!< Transfer mode */
	void (* SetCS) (uint8_t);		/*!< Pointer function to set CS pin output state.
									Must be NULL if CS pin would be handled manually */
} spiConfig_t;

/*****************************************************************************
 * Public functions definitions
 ****************************************************************************/

/**
 * @brief		Initialize SPI module with the corresponding configuration
 * @param[in]	spi Structure with the module configuration
 * @return		1 when success, 0 when fails
 * @note
 */
uint8_t SpiInit(spiConfig_t spi);

/**
 * @brief		Read data from SPI port
 * @param[in]	port SPI Port to read from
 * @param[out]	rx_buffer pointer to buffer where data is stored
 * @param[in]	rx_buffer_size numbers of bytes to read
 * @return  	None
 * @note
 */
void SpiRead(spiPort_t port, uint8_t * rx_buffer, uint32_t rx_buffer_size);

/**
 * @brief		Write data to SPI port
 * @param[in]	port SPI Port to write to
 * @param[out]	tx_buffer pointer to buffer where data is stored
 * @param[in]	tx_buffer_size numbers of bytes to write
 * @return  	None
 * @note
 */
void SpiWrite(spiPort_t port, uint8_t * tx_buffer, uint32_t tx_buffer_size);

/**
 * @brief		Write and Read data simultaneous from SPI port
 * @param[in]	port SPI Port to write to
 * @param[out]	tx_buffer pointer to buffer where data to write is stored
 * @param[out]	rx_buffer pointer to buffer where data read is stored
 * @param[in]	buffer_size numbers of bytes to read or write
 * @return  	None
 * @note
 */
void SpiReadWrite(spiPort_t port, uint8_t * tx_buffer, uint8_t * rx_buffer, uint32_t buffer_size);

/**
 * @brief		Get the transfer status of SPI port
 * @param[in]	port SPI Port to get status
 * @return  	1 when free, 0 when busy
 * @note
 */
uint8_t SpiFree(spiPort_t port);

/**
 * @brief		De-Initialize SPI module with the corresponding configuration
 * @param[in]	spi Structure with the module configuration
 * @return		1 when success, 0 when fails
 * @note
 */
uint8_t SpiDeInit(spiPort_t port);

#endif /* SPI_H_ */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
