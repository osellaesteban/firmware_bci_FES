/**
 * @file NedoPixel.c
 * @author  Franco Cipriani
 * @date 10/11/2018
 * @version 1.0
 */

/*==================[inclusions]=============================================*/
#include "NeoPixel.h"
#include "ws2812b.h"

/*==================[variables]==============================================*/

bit_send data_send;


/*==================[functions]==============================================*/
void NeoPixelInit(uint8_t pin_gpio, uint32_t len)
{
	WS2812bInit(pin_gpio);
	data_send.len_number = len;
	StopWatch_DelayMs(100);
	NeoPixelAllOFF();
	//static bit_send data_save[pin_number];
}


void NeoPixelAllOFF(void)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
	{
		data_send.G = 0;
		data_send.R = 0;
		data_send.B = 0;

		Ws2812bSend(data_send);
	}
	Ws2812bSendRet();
}


void NeoPixelAll(uint8_t color_g, uint8_t color_r, uint8_t color_b)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
	{
		data_send.G = color_g;
		data_send.R = color_r;
		data_send.B = color_b;

		Ws2812bSend(data_send);
	}
	Ws2812bSendRet();
}

void NeoPixelAllGreen(uint8_t brightness)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
		{
			data_send.G = brightness;
			data_send.R = 0;
			data_send.B = 0;

			Ws2812bSend(data_send);
		}
	Ws2812bSendRet();
}

void NeoPixelAllRed(uint8_t brightness)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
	{
		data_send.G = 0;
		data_send.R = brightness;
		data_send.B = 0;
		Ws2812bSend(data_send);
	}
	Ws2812bSendRet();
}


void NeoPixelAllBlue(uint8_t brightness)
{
		for (uint8_t i = 0; i <= data_send.len_number; i++)
			{

			data_send.G = 0;
			data_send.R = 0;
			data_send.B = brightness;

			Ws2812bSend(data_send);
		}
	Ws2812bSendRet();
}




void NeoPixelSingleLed(uint8_t color_g, uint8_t color_r, uint8_t color_b, uint32_t led_number, uint8_t ret)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
	{
		if( i == led_number)
		{

			data_send.G = color_g;
			data_send.R = color_r;
			data_send.B = color_b;
			Ws2812bSend(data_send);
		}
		else
		{
			data_send.G = 0;
			data_send.R = 0;
			data_send.B = 0;
			Ws2812bSend(data_send);
		}
	}
	if(ret == 1)
	{
		//Ws2812bSendRet();
	}

}

void NeoPixelVector(pixel * vector)
{
	for (uint8_t i = 0; i <= data_send.len_number; i++)
	{
		data_send.G = vector[i].G;
		data_send.R = vector[i].R;
		data_send.B = vector[i].B;
		Ws2812bSend(data_send);
	}
}

void NeoPixelAllSameColor(uint8_t color_r, uint8_t color_g, uint8_t color_b, uint8_t leds_bright)
{
		for (uint8_t i = 0; i <= data_send.len_number; i++)
			{
			data_send.G = color_g;
			data_send.R = color_r;
			data_send.B = color_b;
			Ws2812bSend(data_send);
		}
	Ws2812bSendRet();
}

 void NeoPixelSetLen(uint32_t len)
 {
	 data_send.len_number = len;
 }

 uint32_t NeoPixelGetLen(void)
 {
	 return data_send.len_number;
 }



 // TESTS

void NeoPixelTest(void)
{
 	for(uint8_t i = 0 ; i < data_send.len_number; i++)
 	{
 		NeoPixelAllGreen(Max);
 		StopWatch_DelayMs(500);
 		NeoPixelAllRed(Max);
 		StopWatch_DelayMs(500);
 		NeoPixelAllBlue(Max);
 		StopWatch_DelayMs(500);

 		NeoPixelAll(Max,0,Max);
 		StopWatch_DelayMs(500);
 		NeoPixelAll(0,Max,Max);
 		StopWatch_DelayMs(500);
 		NeoPixelAll(Max,Max,0);
 		StopWatch_DelayMs(500);
 		NeoPixelAll(Max,Max,Max);
 		StopWatch_DelayMs(500);
 	}
 	NeoPixelAllOFF();
 	StopWatch_DelayMs(1000);

 	for(uint8_t i = 0 ; i <= 5 ; i++)
 	{
 		for(uint8_t j = 0 ; j <= data_send.len_number ; j++)
 		{
 			NeoPixelSingleLed(50, 50, 50, j,1);
 			StopWatch_DelayMs(250);
 		}
 	NeoPixelAllOFF();
 	}
 	StopWatch_DelayMs(1000);
 }

void NeoPixelTest2(void)
{
	for(uint8_t i = 5; i != Max; i++)
	{
		NeoPixelAllRed(i);
		StopWatch_DelayMs(5);
	}
	for(uint8_t i = Max; i != 5; i--)
	{
		NeoPixelAllRed(i);
		StopWatch_DelayMs(5);
	}
	for(uint8_t i = 5; i != Max; i++)
	{
		NeoPixelAllGreen(i);
		StopWatch_DelayMs(5);
	}
	for(uint8_t i = Max; i != 5; i--)
	{
		NeoPixelAllGreen(i);
		StopWatch_DelayMs(5);
	}
	for(uint8_t i = 5; i != Max; i++)
	{
		NeoPixelAllBlue(i);
		StopWatch_DelayMs(5);
	}
	for(uint8_t i = Max; i != 0; i--)
	{
		NeoPixelAllBlue(i);
		StopWatch_DelayMs(5);
	}
}

void NeoPixelTest3(void)
{
			NeoPixelAllGreen(255);
			StopWatch_DelayMs(1000);
			NeoPixelAllOFF();
			StopWatch_DelayMs(1000);


			NeoPixelAllRed(255);
			StopWatch_DelayMs(1000);
			NeoPixelAllOFF();
			StopWatch_DelayMs(1000);


			NeoPixelAllBlue(255);
			StopWatch_DelayMs(1000);
			NeoPixelAllOFF();
			StopWatch_DelayMs(1000);
}

void NeoPixelTest4(void)
{
	for(uint8_t j = 0; j<= 2 ; j++)
	{
			NeoPixelSingleLed(0,255,0,0,1);
			StopWatch_DelayMs(5000);

			for(uint8_t i = 0; i<= 5 ; i++)
			{
				NeoPixelSingleLed(255,255,0,1,1);
				StopWatch_DelayMs(500);
				NeoPixelAllOFF();
				StopWatch_DelayMs(500);
			}

			NeoPixelSingleLed(255,0,0,2,1);
			StopWatch_DelayMs(5000);

			for(uint8_t i = 0; i<= 5 ; i++)
			{
				NeoPixelSingleLed(255,255,0,1,1);
				StopWatch_DelayMs(500);
				NeoPixelAllOFF();
				StopWatch_DelayMs(500);
			}
	}
}


void NeoPixelTestWs2812bTest(void)
{
	Ws2812bTest();
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////// APP ///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void NeoPixelRight(uint8_t Op)
{
	uint8_t i;
	if(Op == 0)  // Bicicleta o Moto ( 43 Leds)
	{
		//Anillo 24
		for(i=0; i <= Anillo_Max2 ; i++)  // Amarillo Maximo
		{

						data_send.G = Max;
						data_send.R = Max;
						data_send.B = 0;
						Ws2812bSend(data_send);
		}
		for(i=0; i <= Anillo_Max2 ;i++) // Rojo Medio
		{

						data_send.G = 0;
						data_send.R = Med;
						data_send.B = 0;
						Ws2812bSend(data_send);
		}
		//Anillo 16
		for(i=0; i <= Anillo_Med2 ;i++)// Amarillo Maximo
		{

						data_send.G = Max;
						data_send.R = Max;
						data_send.B = 0;
						Ws2812bSend(data_send);
		}
		for(i=0; i <= Anillo_Med2 ;i++)// Rojo Medio
		{

						data_send.G = 0;
						data_send.R = Med;
						data_send.B = 0;
						Ws2812bSend(data_send);
		}
		//Anillo 3
		for(i=0; i <= Anillo_Min ;i++) // Rojo Medio
		{

						data_send.G = 0;
						data_send.R = Med;
						data_send.B = 0;
						Ws2812bSend(data_send);
		}
	}
	else // Auto ( 86 Leds )
	{
		for(uint8_t k =0; k < 2 ;k++)
		{
		//Anillo 24
				for(i=0; i <= Anillo_Max2 ; i++)  // Amarillo Maximo
				{

								data_send.G = Max;
								data_send.R = Max;
								data_send.B = 0;
								Ws2812bSend(data_send);
											}
				for(i=0; i <= Anillo_Max2 ;i++) // Rojo Medio
				{

								data_send.G = 0;
								data_send.R = Med;
								data_send.B = 0;
								Ws2812bSend(data_send);
				}

				//Anillo 16
				for(i=0; i <= Anillo_Med2 ;i++)// Amarillo Maximo
								{

												data_send.G = Max;
												data_send.R = Max;
												data_send.B = 0;
												Ws2812bSend(data_send);
								}
				for(i=0; i <= Anillo_Med2 ;i++)// Rojo Medio
				{

								data_send.G = 0;
								data_send.R = Med;
								data_send.B = 0;
								Ws2812bSend(data_send);
				}

				//Anillo 3
				for(i=0 ; i <= Anillo_Min ;i++) // Rojo Medio
				{

								data_send.G = 0;
								data_send.R = Med;
								data_send.B = 0;
								Ws2812bSend(data_send);
				}
		}
	}
}

void NeoPixelLeft(uint8_t Op)
{
	uint8_t i;
		if(Op == 0)  // Bicicleta o Moto ( 43 Leds)
		{
			//Anillo 24
			for(i=0; i <= Anillo_Max2 ;i++) // Rojo Medio
			{

							data_send.G = 0;
							data_send.R = Med;
							data_send.B = 0;
							Ws2812bSend(data_send);
			}
			for(i=0; i <= Anillo_Max2 ; i++)  // Amarillo Maximo
			{

							data_send.G = Max;
							data_send.R = Max;
							data_send.B = 0;
							Ws2812bSend(data_send);
			}

			//Anillo 16
			for(i=0; i <= Anillo_Med2 ;i++)// Rojo Medio
			{

							data_send.G = 0;
							data_send.R = Med;
							data_send.B = 0;
							Ws2812bSend(data_send);
			}
			for(i=0; i <= Anillo_Med2 ;i++)// Amarillo Maximo
			{

							data_send.G = Max;
							data_send.R = Max;
							data_send.B = 0;
							Ws2812bSend(data_send);
			}

			//Anillo 3
			for(i=0; i <= Anillo_Min ;i++) // Rojo Medio
			{

							data_send.G = 0;
							data_send.R = Med;
							data_send.B = 0;
							Ws2812bSend(data_send);
			}
		}
		else // Auto ( 86 Leds )
		{
			for(uint8_t k =0; k < 2 ;k++)
			{
					//Anillo 24
					for(i=0; i <= Anillo_Max2 ;i++) // Rojo Medio
					{

									data_send.G = 0;
									data_send.R = Med;
									data_send.B = 0;
									Ws2812bSend(data_send);
					}
					for(i=0; i <= Anillo_Max2 ; i++)  // Amarillo Maximo
					{

									data_send.G = Max;
									data_send.R = Max;
									data_send.B = 0;
									Ws2812bSend(data_send);
												}


					//Anillo 16
					for(i=0; i <= Anillo_Med2 ;i++)// Rojo Medio
					{

									data_send.G = 0;
									data_send.R = Med;
									data_send.B = 0;
									Ws2812bSend(data_send);
					}
					for(i=0; i <= Anillo_Med2 ;i++)// Amarillo Maximo
					{

									data_send.G = Max;
									data_send.R = Max;
									data_send.B = 0;
									Ws2812bSend(data_send);
					}


					//Anillo 3
					for(i=0 ; i <= Anillo_Min ;i++) // Rojo Medio
					{

									data_send.G = 0;
									data_send.R = Med;
									data_send.B = 0;
									Ws2812bSend(data_send);
					}
			}
		}

}

void NeoPixelStop (void)
{
	NeoPixelAllRed(Max); // Rojo Maximo
}

void NeoPixelPosition (void)
{
	NeoPixelAllRed(Med); // Rojo Medio
}

void NeoPixelReverse (uint8_t Op)
{
	uint8_t i;
	// Lado Derecho
	//Anillo 24 - Blanco
	for(i=0; i <= Anillo_Max ;i++)
	{
		data_send.G = Max;
		data_send.R = Max;
		data_send.B = Max;
		Ws2812bSend(data_send);
	}
	//Anillo 16 - Rojo Medio
	for(i=0; i <= Anillo_Med ;i++)
	{

		data_send.G = 0;
		data_send.R = Med;
		data_send.B = 0;
		Ws2812bSend(data_send);
	}
	//Anillo 3 - Blanco
	for(i=0; i <= Anillo_Min ;i++)
	{
		data_send.G = Max;
		data_send.R = Max;
		data_send.B = Max;
		Ws2812bSend(data_send);
	}

	if(Op != 0) // Auto
	{
		for(i=0; i <= Anillo_Max ;i++)
		{
			data_send.G = Max;
			data_send.R = Max;
			data_send.B = Max;
			Ws2812bSend(data_send);
		}
		//Anillo 16 - Rojo Medio
		for(i=0; i <= Anillo_Med ;i++)
		{
			data_send.G = 0;
			data_send.R = Med;
			data_send.B = 0;
			Ws2812bSend(data_send);
		}
		//Anillo 3 - Blanco
		for(i=0; i <= Anillo_Min ;i++)
		{
			data_send.G = Max;
			data_send.R = Max;
			data_send.B = Max;
			Ws2812bSend(data_send);
		}
	}

}

void NeoPixelFog (uint8_t Op)
{
	uint8_t i;
	// Lado Derecho
	//Anillo 24 - Rojo Medio
	for(i=0; i <= Anillo_Max ;i++)
	{
		data_send.G = 0;
		data_send.R = Med;
		data_send.B = 0;
		Ws2812bSend(data_send);
	}
	//Anillo 16 - Rojo Medio
	for(i=0; i <= Anillo_Med ;i++)
	{
		data_send.G = 0;
		data_send.R = Med;
		data_send.B = 0;
		Ws2812bSend(data_send);
	}
	//Anillo 3 -Rojo Maximo
	for(i=0; i <= Anillo_Min ;i++)
	{
		data_send.G = 0;
		data_send.R = Max;
		data_send.B = 0;
		Ws2812bSend(data_send);
	}

	if(Op != 0) // Auto
	{
		//Anillo 24 - Rojo Medio
		for(i=0; i <= Anillo_Max ;i++)
		{
			data_send.G = 0;
			data_send.R = Med;
			data_send.B = 0;
			Ws2812bSend(data_send);
		}
		//Anillo 16 - Rojo Medio
		for(i=0; i <= Anillo_Med ;i++)
		{
			data_send.G = 0;
			data_send.R = Med;
			data_send.B = 0;
			Ws2812bSend(data_send);
		}
		//Anillo 3 -Rojo Maximo
		for(i=0; i <= Anillo_Min ;i++)
		{
			data_send.G = 0;
			data_send.R = Max;
			data_send.B = 0;
			Ws2812bSend(data_send);
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Funciones para 3 leds

void NeoPixelRight_For3 (void)
{
	// Amarillo
	data_send.G = Max;
	data_send.R = Max;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Rojo Medio
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Rojo Medio
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);

}

void NeoPixelLeft_For3 (void)
{
	// Rojo Medio
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Rojo Medio
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Amarillo
	data_send.G = Max;
	data_send.R = Max;
	data_send.B = 0;
	Ws2812bSend(data_send);
}

void NeoPixelReverse_For3 (void)
{
	// Blanco
	data_send.G = Max;
	data_send.R = Max;
	data_send.B = Max;
	Ws2812bSend(data_send);
	// Blanco
	data_send.G = Max;
	data_send.R = Max;
	data_send.B = Max;
	Ws2812bSend(data_send);
	// Blanco
	data_send.G = Max;
	data_send.R = Max;
	data_send.B = Max;
	Ws2812bSend(data_send);
}

void NeoPixelFog_For3 (void)
{
	// Rojo Medio
	//data_send.G = 0;
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Rojo Max
	data_send.G = 0;
	data_send.R = Max;
	data_send.B = 0;
	Ws2812bSend(data_send);
	// Rojo Medio
	data_send.G = 0;
	data_send.R = Med;
	data_send.B = 0;
	Ws2812bSend(data_send);
}
