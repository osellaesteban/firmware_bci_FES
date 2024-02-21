/*
 * dsp_EMG.c
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */
#include "dsp_EMG.h"
#include <stdlib.h>

#define APP_FFT_LEN 64
#define SAMPLE_RATE 2000

//Vector de frecuencia para una frecuencia de muestreo de 2khz y un buffer de 128 muestras.
const float freq_vector[65] = { 0.00000000f, 15.62500000f, 31.25000000f,
		46.87500000f, 62.50000000f, 78.12500000f, 93.75000000f, 109.37500000f,
		125.00000000f, 140.62500000f, 156.25000000f, 171.87500000f,
		187.50000000f, 203.12500000f, 218.75000000f, 234.37500000f,
		250.00000000f, 265.62500000f, 281.25000000f, 296.87500000f,
		312.50000000f, 328.12500000f, 343.75000000f, 359.37500000f,
		375.00000000f, 390.62500000f, 406.25000000f, 421.87500000f,
		437.50000000f, 453.12500000f, 468.75000000f, 484.37500000f,
		500.00000000f, 515.62500000f, 531.25000000f, 546.87500000f,
		562.50000000f, 578.12500000f, 593.75000000f, 609.37500000f,
		625.00000000f, 640.62500000f, 656.25000000f, 671.87500000f,
		687.50000000f, 703.12500000f, 718.75000000f, 734.37500000f,
		750.00000000f, 765.62500000f, 781.25000000f, 796.87500000f,
		812.50000000f, 828.12500000f, 843.75000000f, 859.37500000f,
		875.00000000f, 890.62500000f, 906.25000000f, 921.87500000f,
		937.50000000f, 953.12500000f, 968.75000000f, 984.37500000f,
		1000.00000000f };


float32_t dsp_emg_rms_f32(float32_t* pSrc, uint32_t blockSize) {
	float32_t result;
	arm_rms_f32(pSrc, blockSize, &result);
	return result;
}

float32_t dsp_emg_power_f32(float32_t* pSrc, uint32_t blockSize) {
	float32_t aux;
	arm_power_f32(pSrc, blockSize, &aux);
	aux = aux / (blockSize);
	return aux;
}

float32_t dsp_emg_ptp_f32(float32_t* pSrc, uint32_t blockSize) {
	uint32_t idmax, idmin;
	float32_t max, min, ret;
	arm_max_f32(pSrc, blockSize, &max, &idmax);
	arm_min_f32(pSrc, blockSize, &min, &idmin);
	ret = fabsf(max - min);
	return ret;
}

float32_t dsp_emg_iemg_f32(float32_t* pSrc, uint32_t blockSize) {
	//The functions support in-place computation allowing the source and destination pointers to reference the same memory buffer.
	float result = 0;
	uint32_t i = 0;
	if (blockSize > 0) {
		while (i < blockSize) {
			result += fabsf(pSrc[i]);
			i++;
		}
	}
	return result;
}

float32_t dsp_emg_mdf_f32(float32_t * pSrc, uint32_t blockSize) {

	float32_t aux, half_power;
	float32_t data_local[blockSize];
	arm_copy_f32(pSrc, data_local, blockSize);

	arm_rfft_fast_instance_f32 rfft_fast_instance;
	arm_rfft_fast_init_f32(&rfft_fast_instance, blockSize);
	arm_rfft_fast_f32(&rfft_fast_instance, data_local, data_local, 0);
	arm_cmplx_mag_f32(data_local, data_local, APP_FFT_LEN); //fft IS OK.
	arm_mult_f32(data_local, data_local, data_local, APP_FFT_LEN);
	arm_scale_f32(data_local, 1.0f /(blockSize * SAMPLE_RATE), data_local, APP_FFT_LEN);
	arm_scale_f32(&data_local[1], 2.0f, &data_local[1], APP_FFT_LEN - 1); //Power spectrum is ok
	/**@link https://la.mathworks.com/help/signal/ug/power-spectral-density-estimates-using-fft.html */

	//53 is a magic number, FFT is divergent after this value. Why? Nobody knows. But bug in code is possible.
	int magic_number = 53, var = 0;
	aux = 0.0f;
	arm_mean_f32(data_local, magic_number, &half_power);
	half_power = (half_power * magic_number)/2.0; //half power

	while(aux < half_power) {
		aux += data_local[var];
		var++;
	}

	// Lineal interpolation only for simplicity. The CMSIS interpolation needs more memory
	//	((x1-x0)/(y1 - y0))*(y - y0) + x0 = x
	// + (half_power - data_local[var])*(15.625f/(data_local[var+1] - data_local[var]));

	return freq_vector[var];
}

float32_t dsp_emg_mnf_f32(float32_t * pSrc, uint32_t blockSize) {
	float32_t aux1, aux2;
	float32_t data_local[blockSize];
	arm_copy_f32(pSrc, data_local, blockSize);

	arm_rfft_fast_instance_f32 rfft_fast_instance;
	arm_rfft_fast_init_f32(&rfft_fast_instance, blockSize);
	arm_rfft_fast_f32(&rfft_fast_instance, data_local, data_local, 0);
	arm_cmplx_mag_f32(data_local, data_local, APP_FFT_LEN); //fft IS OK.
	arm_mult_f32(data_local, data_local, data_local, APP_FFT_LEN);
	arm_scale_f32(data_local, 1.0f /(blockSize * SAMPLE_RATE), data_local, APP_FFT_LEN);
	arm_scale_f32(&data_local[1], 2.0f, &data_local[1], APP_FFT_LEN - 1); //Power spectrum is ok
	/**@link https://la.mathworks.com/help/signal/ug/power-spectral-density-estimates-using-fft.html */
	/*
	 arm_cfft_f32(&arm_cfft_sR_f32_len64, data_local, 0, 1);
	 arm_cmplx_mag_f32(data_local, data_local, APP_FFT_LEN);
	 arm_mult_f32(data_local, data_local, data_local, APP_FFT_LEN);
	 arm_scale_f32(data_local, 1.0f / (float32_t) (blockSize * SAMPLE_RATE),
	 data_local, APP_FFT_LEN);
	 arm_scale_f32(&data_local[1], 2.0f, &data_local[1], APP_FFT_LEN - 1); //Power spectrum is ok
	 */
	int magic_number = 53;//53 is a magic number fft, is divergent after this value.
	arm_dot_prod_f32(data_local, freq_vector, magic_number, &aux1); //calculate using the dot product
	arm_mean_f32(data_local, magic_number, &aux2);
	aux2 = aux2 * (magic_number);
	aux1 = aux1 / aux2;
	return aux1;
}
