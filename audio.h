#ifndef AUDIO_H
#define AUDIO_H

/* Library includes. */
#include <arm_math.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Peripheral includes. */
#include "stm32f4_discovery.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

/*-----------------------------------------------------------*/

#define ADC_SAMPLES                                 128U
#define FFT_SIZE                                    (ADC_SAMPLES / 2)
#define SAMPLING_FREQUENCY                          44100U
#define SAMPLING_COMPUTATION_TIME_TIM9_CYCLES       100U

/*-----------------------------------------------------------*/

void vInitAudio( void );

#endif