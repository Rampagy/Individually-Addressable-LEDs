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
#define configPATTERN_TASK_TIME_MS                                  ( 20 )
#define configRAINBOW_CROSSFADE_TIME_MS                             ( 30000 )

/* Rainbow crossfade defines. */
#define configRAINBOW_TRANSITION_LENGTH                             ( 30 )
#define configRAINBOW_RAMP_TIME_MS                                  ( 50 )

/*-----------------------------------------------------------*/

/* Patterns */
typedef enum {
    OFF,
    RGB_RAMP
} patterns_t;

/* Color Channels */
typedef enum {
    GRN,
    RED,
    BLU
} colors_t;


/*-----------------------------------------------------------*/

/* Task for creating patterns for the LED strip. */
void vCreatePattern( void * );

/* Fill the strip with a color. */
void vFillStrip( uint8_t R, uint8_t G, uint8_t B );

/* Set a single led. */
void vSetLed( int16_t LED, uint8_t R, uint8_t G, uint8_t B );

/* Get led color channel. */
uint8_t ucGetLed( int16_t LED, uint8_t color );

/* Crossfade between the rainbow colors. */
void vRainbowCrossfade( void );

/* Calcualte the crossfade operation. */
void vCrossfade( int16_t start, uint16_t len, uint8_t ramp, uint8_t R, uint8_t G,uint8_t B );

/*-----------------------------------------------------------*/

/* Get LED buffer from other file (leds.c). */
extern volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS];

/*-----------------------------------------------------------*/

#endif