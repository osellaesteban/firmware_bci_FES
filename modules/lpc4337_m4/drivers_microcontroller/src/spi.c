/** @file spi.c
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

#include "spi.h"
#include "chip.h"

/*****************************************************************************
 * Private macros/types/enumerations/variables definitions
 ****************************************************************************/

#define SUCCESS	1			/* */
#define ERROR 	0			/* */
#define HIGH 	1
#define LOW		0
#define BITS8	8

#define PORT_MISO1	0x1		/*!< SSP1 MISO at pin P1.3 */
#define PIN_MISO1	0x3
#define PORT_MOSI1 	0x1		/*!< SSP1 MOSI at pin P1.4 */
#define PIN_MOSI1 	0x4
#define PORT_SCK1 	0xF		/*!< SSP1 SCK at pin PF.4 */
#define PIN_SCK1 	0x4

/*! SPI1 global variables */
static Chip_SSP_DATA_SETUP_T ssp1_data;		/*!< Data setup structure */
static transferMode_t ssp1_transfer_mode;
static uint8_t ssp1_transfer_completed = TRUE;	/*!< Store port data transfer status */
static uint8_t ssp1_dma_rx_completed;			/*!< Store dma data receive status */
static uint8_t ssp1_dma_tx_completed;			/*!< Store dma data transmit status */
static uint8_t dma_ch_ssp1_tx;					/*!< DMA channel for SSP tx */
static uint8_t dma_ch_ssp1_rx;					/*!< DMA channel for SSP rx */
void (* SetCS1) (uint8_t);				/*!< Pointer function to set SPI1 CS pin state */

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

uint8_t SpiInit(spiConfig_t spi)
{
	uint8_t ret_value;

	switch(spi.port)
	{
	case SPI_1:
		/* Wait until SPI port is free */
		while(!ssp1_transfer_completed);
		/* Configure SSP1 pins */
		Chip_SCU_PinMuxSet(PORT_SCK1, PIN_SCK1, (SCU_MODE_PULLUP | SCU_MODE_FUNC0));
		Chip_SCU_PinMuxSet(PORT_MISO1, PIN_MISO1, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS	| SCU_MODE_FUNC5));
		Chip_SCU_PinMuxSet(PORT_MOSI1, PIN_MOSI1, (SCU_MODE_PULLUP | SCU_MODE_FUNC5));
		/* Configure function to set CS pin state */
		SetCS1 = spi.SetCS;
		/* Initialize and Configure SSP Peripheral */
		Chip_SSP_Init(LPC_SSP1);
		Chip_SSP_SetMaster(LPC_SSP1, spi.mode);
		switch(spi.clk_mode)
		{
		case MODE0:
			Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL0);
			break;

		case MODE1:
			Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA1_CPOL0);
			break;

		case MODE2:
			Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA0_CPOL1);
			break;

		case MODE3:
			Chip_SSP_SetFormat(LPC_SSP1, SSP_BITS_8, SSP_FRAMEFORMAT_SPI, SSP_CLOCK_CPHA1_CPOL1);
			break;
		}
		Chip_SSP_SetBitRate(LPC_SSP1, spi.bitrate);
		Chip_SSP_Enable(LPC_SSP1);
		ssp1_transfer_mode = spi.transfer_mode;
		switch(spi.transfer_mode)
		{
		case SPI_POLLING:
			break;

		case SPI_INTERRUPT:
			/* Setting SSP interrupt */
			NVIC_EnableIRQ(SSP1_IRQn);
			break;

		case SPI_DMA:
			/* Initialize GPDMA controller */
			Chip_GPDMA_Init(LPC_GPDMA);
			/* Setting GPDMA interrupt */
			NVIC_DisableIRQ(DMA_IRQn);
			NVIC_SetPriority(DMA_IRQn, ((0x01 << 3) | 0x01));
			NVIC_EnableIRQ(DMA_IRQn);
			break;
		}

		ret_value = SUCCESS;
		break;

	default:
		ret_value = ERROR;
		break;
	}

	return ret_value;
}

void SpiRead(spiPort_t port, uint8_t * rx_buffer, uint32_t rx_buffer_size)
{
	switch(port)
	{
	case SPI_1:
		/* Wait until SPI port is free */
		while(!ssp1_transfer_completed);
		ssp1_transfer_completed = FALSE;
		/* If CS controlled by driver, activate CS */
		if (SetCS1 != NULL)
		{
			SetCS1(LOW);
		}

		switch(ssp1_transfer_mode)
		{
		case SPI_POLLING:
			/* Initialize data setup structure */
			ssp1_data.tx_data = NULL;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = rx_buffer;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = rx_buffer_size;
			Chip_SSP_RWFrames_Blocking(LPC_SSP1, &ssp1_data);
			ssp1_transfer_completed = TRUE;
			/* If CS controlled by driver, deactivate CS */
			if (SetCS1 != NULL)
			{
				SetCS1(HIGH);
			}
			break;

		case SPI_INTERRUPT:
			/* Initialize data setup structure */
			ssp1_data.tx_data = NULL;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = rx_buffer;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = rx_buffer_size;
			/* flush dummy data from SSP FiFO */
			Chip_SSP_Int_FlushData(LPC_SSP1);
			/* transmit first byte */
			Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &ssp1_data);
			/* enable interrupt */
			Chip_SSP_Int_Enable(LPC_SSP1);
			break;

		case SPI_DMA:
			ssp1_dma_rx_completed = FALSE;
			ssp1_dma_tx_completed = TRUE;
			/* Get DMA channels for rx */
			dma_ch_ssp1_rx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Rx);
			Chip_SSP_DMA_Enable(LPC_SSP1);
			/* data SSP --> rx_buffer */
			Chip_GPDMA_Transfer(LPC_GPDMA, dma_ch_ssp1_rx, GPDMA_CONN_SSP1_Rx, (uint32_t) &rx_buffer[0],
				GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA, rx_buffer_size);
			break;
		}
		break;
	}
}

void SpiWrite(spiPort_t port, uint8_t * tx_buffer, uint32_t tx_buffer_size)
{
	switch(port)
	{
	case SPI_1:
		/* Wait untill SPI port is free */
		while(!ssp1_transfer_completed);
		ssp1_transfer_completed = FALSE;
		/* If CS controlled by driver, activate CS */
		if (SetCS1 != NULL)
		{
			SetCS1(LOW);
		}

		switch(ssp1_transfer_mode)
		{
		case SPI_POLLING:
			/* Initialize data setup structure */
			ssp1_data.tx_data = tx_buffer;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = NULL;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = tx_buffer_size;
			Chip_SSP_RWFrames_Blocking(LPC_SSP1, &ssp1_data);
			ssp1_transfer_completed = TRUE;
			/* If CS controlled by driver, deactivate CS */
			if (SetCS1 != NULL)
			{
				SetCS1(HIGH);
			}
			break;

		case SPI_INTERRUPT:
			/* Initialize data setup structure */
			ssp1_data.tx_data = tx_buffer;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = NULL;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = tx_buffer_size;
			/* flush dummy data from SSP FiFO */
			Chip_SSP_Int_FlushData(LPC_SSP1);
			/* transmit first byte */
			Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &ssp1_data);
			/* enable interrupt */
			Chip_SSP_Int_Enable(LPC_SSP1);
			break;

		case SPI_DMA:
			ssp1_dma_rx_completed = TRUE;
			ssp1_dma_tx_completed = FALSE;
			/* Get DMA channels for tx */
			dma_ch_ssp1_tx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Tx);
			Chip_SSP_DMA_Enable(LPC_SSP1);
			/* data tx_buffer --> SSP */
			Chip_GPDMA_Transfer(LPC_GPDMA, dma_ch_ssp1_tx, (uint32_t) tx_buffer, GPDMA_CONN_SSP1_Tx,
				GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA, tx_buffer_size);
			break;
		}
		break;
	}
}

void SpiReadWrite(spiPort_t port, uint8_t * tx_buffer, uint8_t * rx_buffer, uint32_t buffer_size)
{
	switch(port)
	{
	case SPI_1:
		/* Wait untill SPI port is free */
		while(!ssp1_transfer_completed);
		ssp1_transfer_completed = FALSE;
		/* If CS controlled by driver, activate CS */
		if (SetCS1 != NULL)
		{
			SetCS1(LOW);
		}

		switch(ssp1_transfer_mode)
		{
		case SPI_POLLING:
			/* Initialize data setup structure */
			ssp1_data.tx_data = tx_buffer;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = rx_buffer;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = buffer_size;
			Chip_SSP_RWFrames_Blocking(LPC_SSP1, &ssp1_data);
			ssp1_transfer_completed = TRUE;
			/* If CS controlled by driver, deactivate CS */
			if (SetCS1 != NULL)
			{
				SetCS1(HIGH);
			}
			break;

		case SPI_INTERRUPT:
			/* Initialize data setup structure */
			ssp1_data.tx_data = tx_buffer;
			ssp1_data.tx_cnt = 0;
			ssp1_data.rx_data = rx_buffer;
			ssp1_data.rx_cnt = 0;
			ssp1_data.length = buffer_size;
			/* flush dummy data from SSP FiFO */
			Chip_SSP_Int_FlushData(LPC_SSP1);
			/* transmit first byte */
			Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &ssp1_data);
			/* enable interrupt */
			Chip_SSP_Int_Enable(LPC_SSP1);
			break;

		case SPI_DMA:
			ssp1_dma_rx_completed = FALSE;
			ssp1_dma_tx_completed = FALSE;
			/* Get DMA channels for tx and rx */
			dma_ch_ssp1_tx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Tx);
			dma_ch_ssp1_rx = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_SSP1_Rx);
			/* data tx_buffer --> SSP */
			Chip_SSP_DMA_Enable(LPC_SSP1);
			Chip_GPDMA_Transfer(LPC_GPDMA, dma_ch_ssp1_tx, (uint32_t) &tx_buffer[0], GPDMA_CONN_SSP1_Tx,
				GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA, buffer_size);
			/* data SSP --> rx_buffer */
			Chip_GPDMA_Transfer(LPC_GPDMA, dma_ch_ssp1_rx, GPDMA_CONN_SSP1_Rx, (uint32_t) &rx_buffer[0],
				GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA, buffer_size);
			break;
		}
		break;
	}
}

uint8_t SpiFree(spiPort_t port)
{
	uint8_t transfer_completed;
	switch(port)
		{
		case SPI_1:
			transfer_completed = ssp1_transfer_completed;
			break;
		}
	return transfer_completed;
}

uint8_t SpiDeInit(spiPort_t port)
{
	uint8_t ret_value;

	switch(port)
	{
	case SPI_1:
		Chip_SSP_Disable(LPC_SSP1);
		Chip_SSP_DeInit(LPC_SSP1);
		ret_value = SUCCESS;
		break;

	default:
		ret_value = ERROR;
		break;
	}

	return ret_value;
}

/**
 * @brief	SSP interrupt handler sub-routine
 * @return	Nothing
 */
void SSP1_IRQHandler(void)
{
	/* Disable all interrupt */
	Chip_SSP_Int_Disable(LPC_SSP1);
	Chip_SSP_Int_RWFrames8Bits(LPC_SSP1, &ssp1_data);
	if ((ssp1_data.rx_cnt != ssp1_data.length) || (ssp1_data.tx_cnt != ssp1_data.length))
	{
		/* enable all interrupts */
		Chip_SSP_Int_Enable(LPC_SSP1);
	}
	else
	{
		ssp1_transfer_completed = TRUE;
		/* If CS controlled by driver, deactivate CS */
		if (SetCS1 != NULL)
		{
			SetCS1(HIGH);
		}
	}
}

/**
 * @brief	DMA interrupt handler sub-routine.
 * @return	Nothing
 */
void DMA_IRQHandler(void)
{
	/* Check if DMA transfer is completed */
	if(!ssp1_dma_rx_completed)
	{
		if (Chip_GPDMA_Interrupt(LPC_GPDMA, dma_ch_ssp1_rx) == SUCCESS)
		{
			ssp1_dma_rx_completed = TRUE;
		}
	}
	if(!ssp1_dma_tx_completed)
	{
		if (Chip_GPDMA_Interrupt(LPC_GPDMA, dma_ch_ssp1_tx) == SUCCESS)
		{
			ssp1_dma_tx_completed = TRUE;
		}
	}
	if (ssp1_dma_rx_completed && ssp1_dma_tx_completed)
	{
		ssp1_transfer_completed = TRUE;
		Chip_SSP_DMA_Disable(LPC_SSP1);
		/* If CS controlled by driver, deactivate CS */
		if (SetCS1 != NULL)
		{
			SetCS1(HIGH);
		}
	}
}
