#ifndef PATTERN_H
#define PATTERN_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Peripheral includes. */
#include "leds.h"

/* Project specific defines. */
#define configRGB_RAMP_TIME_MS                                      ( 1000 )

/*-----------------------------------------------------------*/

/* Patterns */
typedef enum {
    OFF,
    RGB_RAMP
} patterns_t;

/*-----------------------------------------------------------*/

/* Task for creating patterns for the LED strip. */
void vCreatePattern( void * );

/* function for turning all Leds off. */
void vTurnLedsOff( void );

/*-----------------------------------------------------------*/

/* Get LED buffer from other file (leds.c). */
extern volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS];

/*-----------------------------------------------------------*/

#endif