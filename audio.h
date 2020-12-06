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

/* User includes. */
#include "debug.h"

/*-----------------------------------------------------------*/

#define ADC_SAMPLES                                 2048U
#define CFFT_SIZE                                   (ADC_SAMPLES / 2)
#define RFFT_SIZE                                   (CFFT_SIZE / 2)
#define SAMPLING_FREQUENCY                          44100U

/*-----------------------------------------------------------*/

/* Initialize peripherals used in audio processing. */
void vInitAudio( void );

/* Function for performing an FFT. */
void vPerformFFT( float32_t* ufFourierFrequency );

/*-----------------------------------------------------------*/

/* Get debug stats from other file (debug.c) */
extern xDebugStats_t xDebugStats;

#endif