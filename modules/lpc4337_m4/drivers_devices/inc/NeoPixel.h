/** \addtogroup Drivers_Programable Drivers Programable
 ** @{ */
/** \addtogroup Drivers_Devices Drivers devices
 ** @{ */
/** \addtogroup NeoPixel
 ** @{ */


/**
 * @file NeoPixel.h
 * @author  Franco Cipriani
 * @date 10/11/2018
 * @version 1.0
 *
 * @brief Library for NeoPixel
 *
 *
 * - - -
 */

#ifndef MODULES_LPC4337_M4_BOARD_INC_NEOPIXEL_H_
#define MODULES_LPC4337_M4_BOARD_INC_NEOPIXEL_H_

/*==================[inclusions]=============================================*/
#include "stdint.h"

/*==================[macros]=================================================*/
#define Max 255
#define Med 30 //126
#define Min 1

//Normal APP
#define Anillo_Max 24
#define Anillo_Max2 12
#define Anillo_Med 16
#define Anillo_Med2 8
#define Anillo_Min 3
/*==================[typedef]================================================*/
typedef struct
{
	 uint8_t R;  		  	//Red
	 uint8_t G;  			//Green
	 uint8_t B;  			//Blue
	 uint8_t bright;  	//Brightness
} pixel;
/*==================[external data declaration]==============================*/
/*==================[external functions declaration]=========================*/


/**
 * @fn void NeoPixelInit(void)
 *
 * @brief Initialization of the driver WS2812b
 *
 * - inicializa el WS2812b, le pasa por parametro el pin a utilizar
 * - configura la cantidad de led que se van autilzar
 *
 *
 * @param[in] pin GPIO pin edu-ciaa ( Connector  Serigraphy )
 * @param[in] len number of leds
 * @return null
 */
void NeoPixelInit(uint8_t pin_gpio, uint32_t len);

/**
 * @fn void NeoPixelAllOFF(void)
 *
 * @brief All off leds
 *
 *
 * @param[in] null
 * @return null
 */
void NeoPixelAllOFF(void);

/**
 * @fn void NeoPixelAll(uint8_t color_r, uint8_t color_g, uint8_t color_b)
 *
 * @brief  Turn on all the LEDs of a specific color
 *
 * @param[in] color_R Set color for Led Red
 * @param[in] color_G Set color for Led Gren
 * @param[in] color_B Set color for Led Blue
 * @return null
 */
void NeoPixelAll(uint8_t color_g, uint8_t color_r, uint8_t color_b);

/**
 * @fn void NeoPixelAllgren(void)
 *
 * @brief All leds of color Gren
 *
 * @param[in] brightness
 * @return null
 */
void NeoPixelAllGreen(uint8_t brightness);

/**
 * @fn void NeoPixelAllRed(void)
 *
 * @brief All leds of color Red
 *
 * @param[in] brightness
 * @return null
 */
void NeoPixelAllRed(uint8_t brightness);


/**
 * @fn void NeoPixelAllBlue(void)
 *
 * @brief All leds of color Blue
 *
 * @param[in] brightness
 * @return null
 */
void NeoPixelAllBlue(uint8_t brightness);

/**
 * @fn void void NeoPixelSingleLed(uint8_t color_r, uint8_t color_g, uint8_t color_b, uint8_t led_number)
 *
 * @brief Turn on a specific color led
 *
 * @param[in] color_G Set color for Led Gren
 * @param[in] color_R Set color for Led Red
 * @param[in] color_B Set color for Led Blue
 * @param[in] led_ number number led for turn on
 * @param[in] ret ret on = 1, ret Off != 1
 * @return null
 */
void NeoPixelSingleLed(uint8_t color_g, uint8_t color_r, uint8_t color_b, uint32_t led_number, uint8_t ret);

/**
 * @fn void void NeoPixelSingleLed(uint8_t color_r, uint8_t color_g, uint8_t color_b, uint8_t led_number)
 *
 * @brief Turn on a specific color led
 *
 * @param[in] vector Pixels vector pointer
 * @param[in] ret ret on = 1, ret Off != 1
 * @return null
 */
void NeoPixelVector(pixel * vector);

/**
 * @fn void void NeoPixelAllSameColor(uint8_t color_r, uint8_t color_g, uint8_t color_b, uint8_t led_number)
 *
 * @brief Turn on a specific color led
 *
 * @param[in] color_G Set color for Led Gren
 * @param[in] color_R Set color for Led Red
 * @param[in] color_B Set color for Led Blue
 * @param[in] leds_bright Set same brightness for all leds.
 * @return null
 */
void NeoPixelAllSameColor(uint8_t color_r, uint8_t color_g, uint8_t color_b, uint8_t leds_bright);



/**
 * @fn void NeoPixelTest(void)
 *
 * @brief  Get the amount of led
 *
 *
 * @param[in] null
 * @return amount of led
 */

uint32_t NeoPixelGetLen(void);

/**
 * @fn void NeoPixelTest(void)
 *
 * @brief Set the amount of led
 *
 *
 * @param[in] len New amount of led
 * @return null
 */
 void NeoPixelSetLen(uint32_t len);


 /**
  * @fn void NeoPixelTest(void)
  *
  * @brief  Test for all leds and all functions
  *
  * @param[in] null
  * @return null
  */
 void NeoPixelTest(void);

 /**
  * @fn void NeoPixelTest2(void)
  *
  * @brief  Test for all leds and all functions
  *
  * @param[in] null
  * @return null
  */
 void NeoPixelTest2(void);

 /**
  * @fn void NeoPixelTest3(void)
  *
  * @brief  Test for all leds and all functions
  *
  * @param[in] null
  * @return null
  */
 void NeoPixelTest3(void);
 /**
  * @fn void NeoPixelTestWs2812bTest(void)
  *
  * @brief  Test for driver ws2812b
  *
  * @param[in] null
  * @return null
  */
 void NeoPixelTestWs2812bTest(void);


 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////// APP ///////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



 void NeoPixelRight(uint8_t Op);

 void NeoPixelLeft(uint8_t Op);

 void NeoPixelStop(void);

 void NeoPixelPosition(void);

 void NeoPixelReverse (uint8_t Op);

 void NeoPixelFog(uint8_t Op);

 void NeoPixelRight_For3 (void);

 void NeoPixelLeft_For3 (void);

 void NeoPixelReverse_For3 (void);

 void NeoPixelFog_For3 (void);
/*==================[end of file]============================================*/

 /** @} doxygen end group definition */
 /** @} doxygen end group definition */
 /** @} doxygen end group definition */

#endif /* MODULES_LPC4337_M4_BOARD_INC_NEOPIXEL_H_ */



/**
 * @fn void NeoPixelInit(void);
 *
 * @brief

 *
 *
 * @param[in]
 * @return
 */
