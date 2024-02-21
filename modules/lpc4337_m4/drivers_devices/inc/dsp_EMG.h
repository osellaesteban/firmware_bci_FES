/*
 * dsp_EMG.h
 *
 *  Created on: 15 sept. 2021
 *      Author: river
 */

#ifndef MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_
#define MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_

// DSP libs
#define ARM_MATH_CM4
#define __FPU_PRESENT 1
#include "arm_math.h"
#include "arm_const_structs.h"

/**
 * @brief  Root Mean Square of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult   is output value.
 */
float32_t dsp_emg_rms_f32(float32_t * pSrc, uint32_t blockSize);


/**
 * @brief  Sum of the squares of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult    is output value.
 */
float32_t dsp_emg_power_f32(float32_t * pSrc, uint32_t blockSize);

/**
 * @brief  Pick to pick amplitude of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult    is output value.
 */
float32_t dsp_emg_ptp_f32(float32_t * pSrc, uint32_t blockSize);

/**
 * @brief  Integral EMG of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult    is output value.
 */
float32_t dsp_emg_iemg_f32(float32_t * pSrc, uint32_t blockSize);

/**
 * @brief  MDF of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult   is output value.
 */
float32_t dsp_emg_mdf_f32(float32_t * pSrc, uint32_t blockSize);

/**
 * @brief  MNF of the elements of a floating-point vector.
 * @param[in]  pSrc       is input pointer
 * @param[in]  blockSize  is the number of samples to process
 * @return[out] pResult   is output value.
 */
float32_t dsp_emg_mnf_f32(float32_t * pSrc, uint32_t blockSize);



#endif /* MODULES_LPC4337_M4_DRIVERS_DEVICES_INC_DSP_EMG_H_ */
