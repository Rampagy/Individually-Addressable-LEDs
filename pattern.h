#ifndef PATTERN_H
#define PATTERN_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Peripheral includes. */
#include "leds.h"

/*-----------------------------------------------------------*/

/* Pattern length times. */
#define configPATTERN_TASK_TIME_MS                                  ( 10 )
#define configRAINBOW_CROSSFADE_TIME_MS                             ( 30000 )
#define configAURORA_BOREALIS_TIME_MS                               ( 60000 )
#define configWAVE_TIME_MS                                          ( 30000 )

/*-----------------------------------------------------------*/

/* Rainbow crossfade defines. */
#define configRAINBOW_TRANSITION_LENGTH                             ( 30 )
#define configRAINBOW_RAMP_TIME_MS                                  ( 50 )

/*-----------------------------------------------------------*/

/* Aurora borealis defines. */

/* Number of aurora's to simultaneously show. */
#define configAURORA_BOREALIS_LENGTH                                ( 20 )

/* How fast to move the aurora's. */
#define configAURORA_BOREALIS_DELAY_MS                              ( 50 )

/*-----------------------------------------------------------*/

/* Wave defines. */
#define configWAVE_LENGTH                                           ( 30 )
#define configWAVE_NUM_COLORS                                       ( 4 )
#define configWAVE_COLORS               \
                {255, 0, 255},          \
                {255, 50, 50},          \
                {255, 0, 150},           \
                {150, 0, 255},

/*-----------------------------------------------------------*/

/* Patterns */
typedef enum {
    OFF,
    RGB_RAMP,
    AURORA_BOREALIS,
    WAVE,
    LAST_PATTERN
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

/* Cosine Lookup table function. */
uint8_t ucGetCos( int32_t val );

/* Create a wave pattern. */
void vWave( const uint32_t ulPatternCount );

/* Simulate the aurora borealis. */
void vAuroraBorealis( const uint32_t usPatternCount );

/* Crossfade between the rainbow colors. */
void vRainbowCrossfade( const uint32_t usPatternCount );

/* Calcualte the crossfade operation. */
void vCrossfade( int16_t start, uint16_t len, uint8_t ramp, uint8_t R, uint8_t G,uint8_t B );

/*-----------------------------------------------------------*/

/* Get LED buffer from other file (leds.c). */
extern volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS];

/*-----------------------------------------------------------*/

#endif