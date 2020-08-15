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
void vSetLed( uint16_t LED, uint8_t R, uint8_t G, uint8_t B );

/* Get led color channel. */
uint8_t ucGetLed( uint16_t LED, uint8_t color );

/* Cross fade between the rainbow colors.  */
void vRainbowCrossfade( void );

/*-----------------------------------------------------------*/

/* Get LED buffer from other file (leds.c). */
extern volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS];

/*-----------------------------------------------------------*/

#endif