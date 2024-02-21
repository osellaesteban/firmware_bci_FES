
/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup WS2812
 ** @{ */

/**
 * @file WS2812b.h
 * @author  Franco Cipriani
 * @date 10/11/2018
 * @version 1.0
 *
 * @section genDesc Descripción general
 * @brief General description
 *
 *
 * WS2812B is a intelligent control LED light source that the control circuit and RGB chip are integrated in
 * a package of 5050 components. It internal include intelligent digital port data latch and signal reshaping amplification drive circuit.
 * Also include a precision internal oscillator and a 12V voltage programmable constant current
 * control part, effectively ensuring the pixel point light color height consistent.
 * The data transfer protocol use single NZR communication mode. After the pixel power-on reset, the DIN
 * port receive data from controller, the first pixel collect initial 24bit data then sent to the internal data latch,
 * the other data which reshaping by the internal signal reshaping amplification circuit sent to the next cascade
 * pixel through the DO port. After transmission for each pixel,the signal to reduce 24bit. pixel adopt auto reshaping transmit technology,
 *  making the pixel cascade number is not limited the signal transmission, only depend on the speed of signal transmission.
 * LED with low driving voltage, environmental protection and energy saving, high brightness, scattering angle is large,
 * good consistency, low power, long life and other advantages. The control chip integrated in LED
 * above becoming more simple circuit, small volume, convenient installation.
 *
 *
 * - - -
 *
 * @section Protocol
 *
 * These LED strips are controlled by a simple, high-speed one-wire protocol on the input signal line. The protocol is documented in the WS2812B datasheet (266k pdf) and also below.
 *
 * The default, idle state of the signal line is low. To update the LED colors, you need to transmit a series of high pulses on the signal line.
 * Each high pulse encodes one bit: a short pulse (0.35 μs) represents a zero, while a long pulse (0.9 μs) represents a one.
 * The time between consecutive rising edges should be 1.25 μs (though in our tests, the strips worked with cycle times up to approximately 6 μs).
 * After the bits are sent, the signal line should be held low for 50 μs to send a reset command, which makes the new color data take effect
 * (note: it is possible for low pulses as short as 6 μs to trigger a reset).
 * The pulse widths do not have to be precise: there is a threshold that determines whether the pulse is a 0 or a 1,
 * and a wide range of pulse widths on both sides of the threshold will work.
 *
 * The color of each LED is encoded as three LED brightness values, which must be sent in GRB (green-red-blue) order.
 * Each brightness value is encoded as a series of 8 bits, with the most significant bit being transmitted first, so each LED color takes 24 bits.
 * The first color transmitted applies to the LED that is closest to the data input connector,
 * while the second color transmitted applies to the next LED in the strip, and so on.
 *
 * To update all the LEDs in the strip, you should send all the colors at once with no pauses.
 * If you send fewer colors than the number of LEDs on the strip, then some LEDs near the end of the strip will not be updated.
 * For example, to update all 30 LEDs on a 1-meter strip, you would send 720 bits encoded as high pulses and then hold the signal line low for 50 μs.
 * If multiple strips are chained together with their data connectors, they can be treated as one longer strip and updated the same way
 * (two chained 1-meter strips behave the same as one 2-meter strip).
 *
 * Each RGB LED receives data on its data input line and passes data on to the next LED using its data output line.
 * The high-speed protocol of the WS2812B allows for fast updates; our library for the Arduino below takes about 1.1 ms to update 30 LEDs,
 * so it is possible to update 450 LEDs faster than 60 Hz. However, constant updates are not necessary;
 * the LED strip can hold its state indefinitely as long as power remains connected.
 *
 *
 * - - -
 *
 *	200 Mhz = tick at 5 ns
 *
 *	hay q usar un timer
 *
 * - - -
 *
 * @section genDesc Map pins
 *
 * | Pin |  Name  | Description                     |
 * |:---:|:------:|:----------------------------- --|
 * |  1  |  VDD   | Power supply LED                |
 * |  2  |  DOUT  | Control data signal output      |
 * |  3  |  VSS   | Ground                          |
 * |  4  |  DIN   | DIN Control data signal input   |
 *
 * - - -
 * @section genDesc transfer time
 *
 * Data transfer time( TH+TL=1.25 μs ±600n s )
 * | Data  |  Description                |    time     |  delta  | tick |
 * |:-----:|:---------------------------:|:-----------:|:-------:|:-------:|
 * |  T0H  |  0 code ,high voltage time  |    0.40us   | ±150ns  |
 * |  T1H  |  1 code ,high voltage time  |    0.85us   | ±150ns  |
 * |  T0L  |  0 code ,low voltage time   |    0.85us   | ±150ns  |
 * |  T1L  |  1 code ,low voltage time   |    0.40us   | ±150ns  |
 * |  RES  |  low voltage time           |  Above 50μs |         |
 *
 */

#ifndef MODULES_LPC4337_M4_DRIVERS_BM_INC_WS2812B_H_
#define MODULES_LPC4337_M4_DRIVERS_BM_INC_WS2812B_H_

/*==================[inclusions]=============================================*/
#include "chip.h"
#include "stopwatch.h"

/*==================[macros]=================================================*/
#define Ticks (1/200000000) 	// 1 Ticks = 5 ns

#define T0H 80  		//   80 x 5 ticks =  400 ns
#define T1H 170		//   170 x 5 ticks = 850 ns

#define T0L 170 		//   170 x 5 ticks = 850 ns
#define T1L 80 	    //   80 x 5 ticks =  400 ns

#define RET 50 // us


/*==================[typedef]================================================*/
typedef struct
{
	 uint32_t len_number :8;  	//data null
	 uint32_t G :8;  		  	//Green
	 uint32_t R :8;  			//Red
	 uint32_t B :8;  			//Blue
} bit_send;

typedef struct
{
	uint8_t bit_0;
	uint8_t bit_1;
	uint8_t bit_2;
	uint8_t bit_3;
	uint8_t bit_4;
	uint8_t bit_5;
	uint8_t bit_6;
	uint8_t bit_7;
} bit_n;

typedef struct
{
	uint8_t hwPort ; //hw: HardWard Port
	uint8_t hwPin ;  //HardWard Pin
	uint8_t GpioPort ;
	uint8_t GpioPin ;
	uint16_t modo ;
}  digitalIO;

/*==================[external data declaration]==============================*/
/*==================[external functions declaration]=========================*/

/**
 * @fn void WS2812bInit(uint8_t Pin)
 *
 * @brief Initialization of the WS2812b driver for NeoPixel
 *
 *
 * @param[in] pin : GPIO0 - GPIO8 --> gpioMap_t
 * @return null
 */
void WS2812bInit(uint8_t pin);

/**
 * @fn void Ws2812bSend(void)
 *
 * @brief  1) Save state and disable all Interrupt
 *  	   2) Send data
 *		   3) Return active Interrupt
 *
 * @param[in] data
 * @return null
 */
void Ws2812bSend(bit_send data);

/**
 * @fn void Ws2812bSendHigh(uint8_t pin)
 *
 * @brief Send bit high
 *
 *
 * @param[in] pin out
 * @return null
 */
void Ws2812bSendHigh(uint8_t pin);

/**
 * @fn void Ws2812bSendLow(uint8_t pin)
 *
 * @brief Send bit Low
 *
 *
 * @param[in] pin out
 * @return null
 */
void Ws2812bSendLow(uint8_t pin);

/**
 * @fn void Ws2812bSendRet(uint8_t pin)
 *
 * @brief Send comand Ret
 *
 *
 * @param[in] pin out
 * @return null
 */
void Ws2812bSendRet(void);

/**
 * @fn void Config_GPIO(uint8_t pin)
 *
 * @brief Send bit Low
 *
 *
 * @param[in] pin out
 * @return null
 */
void Ws2812bConfigGPIO(uint8_t pin);

/**
 * @fn void Ws2812bTest(void)
 *
 * @brief Test frecuecia
 *
 *
 * @param[in] null
 * @return null
 */
void Ws2812bTest(void);


/**
 * @fn void Ws2812bInterruptDisable(void)
 *
 * @brief Disable all Interrupt external
 *
 *		Get Active Interrupt, Save, Disable all
 *
 * @param[in] null
 * @return null
 */
void Ws2812bInterruptDisable(void);

/**
 * @fn void Ws2812bInterrupt(void)
 *
 * @brief Return active Interrupt
 *
 *		Set Active Interrupt.
 *
 * @param[in] null
 * @return null
 */
void Ws2812bInterruptActive(void);


void nada(void);
/*==================[end of file]============================================*/
#endif /* MODULES_LPC4337_M4_DRIVERS_BM_INC_WS2812B_H_ */


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
