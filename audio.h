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
#define FFT_SIZE                                    (ADC_SAMPLES / 2)
#define SAMPLING_FREQUENCY                          44100U

/*-----------------------------------------------------------*/

void vInitAudio( void );

/*-----------------------------------------------------------*/

/* Get debug stats from other file (debug.c) */
extern xDebugStats_t xDebugStats;

#endif