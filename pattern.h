#ifndef PATTERN_H
#define PATTERN_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Peripheral includes. */
#include "leds.h"
#include "audio.h"

/*-----------------------------------------------------------*/

/* Pattern length times. */
#define configPATTERN_TASK_TIME_MS                                  ( 10 )
#define configRAINBOW_CROSSFADE_TIME_MS                             ( 45000 )
#define configAURORA_BOREALIS_TIME_MS                               ( 60000 )
#define configLASER_TIME_MS                                         ( 30000 )
#define configFIRE_SPARKS_TIME_MS                                   ( 60000 )
#define configRGB_AUDIO_TIME_MS                                     ( 120000 )

/*-----------------------------------------------------------*/

/* Rainbow crossfade defines. */
#define configRAINBOW_TRANSITION_LENGTH                             ( 5 )
#define configRAINBOW_RAMP_TIME_MS                                  ( 50 )
#define configRAINBOW_SIMULTANEOUS_COLORS                           ( 4 )

/*-----------------------------------------------------------*/

/* Aurora borealis defines. */

/* Number of aurora's to simultaneously show. */
#define configAURORA_BOREALIS_LENGTH                                ( 20 )

/* How fast to move the aurora's. */
#define configAURORA_BOREALIS_DELAY_MS                              ( 50 )

/*-----------------------------------------------------------*/

/* Laser defines. */
#define configLASER_LENGTH                                          ( 30 )
#define configLASER_INCREMENT_DELAY                                 ( 10 )
#define configLASER_NUM_COLORS                                      ( 4 )
#define configLASER_COLORS              \
                {255, 0, 255},          \
                {255, 50, 50},          \
                {255, 0, 150},          \
                {150, 0, 255},

/*-----------------------------------------------------------*/

/*  Fire sparks defines. */
#define configFIRE_SPARKS                                           ( 1 )
#define configFIRE_DIM_SPEED                                        ( 98 )   /* Percentage * 100 , must be <= 100*/
#define configFIRE_DIM_INTERVAL                                     ( 10 )
#define configFIRE_EMBER_INTERVAL                                   ( 20 )
#define configFIRE_EMBER_LENGTH                                     ( 5 )
#define configFIRE_NUM_COLORS                                       ( 6 )
#define configFIRE_COLORS                       \
                {0x80, 0x11, 0x00},             \
                {0xFF, 0x8C, 0x00},             \
                {0xD7, 0x35, 0x02},             \
                {0xFC, 0x64, 0x00},             \
                {0xE8, 0x61, 0x00},             \
                {0xE2, 0x58, 0x22},

/*-----------------------------------------------------------*/

/* RGB audio defines. */
#define configRGB_AUDIO_SECTIONS                                    ( 3 )
#define configRGB_AUDIO_SECTION_COLORS                              400U, 2500U, 10000U
#define configRGB_AUDIO_LEDS_PER_SECTION                            ( NUMBER_OF_LEDS / configRGB_AUDIO_SECTIONS )
#define configRGB_AUDIO_BRIGHTNESS_THRESHOLD                        ( 55 )
#define configRGB_AUDIO_MAX_BRIGHTNESS                              ( 600 )
#define configRGB_AUDIO_FADE_SPEED                                  ( 3 )

/*-----------------------------------------------------------*/

/* Patterns */
typedef enum {
    RAINBOW_CROSSFADE,
    AURORA_BOREALIS,
    LASER,
    FIRE_SPARKS,
    RGB_AUDIO,
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

/* Function for getting the next pattern, if necessary. */
void vCheckNextPattern( uint32_t* ulPatternCount, const uint32_t ulPatternLength, patterns_t* eCurrentPattern );

/* Fill the strip with a color. */
void vFillStrip( uint8_t R, uint8_t G, uint8_t B );

/* Set a single led. */
void vSetLed( int16_t LED, int16_t R, int16_t G, int16_t B );

/* Get led color channel. */
uint8_t ucGetLed( int16_t LED, uint8_t color );

/* Cosine Lookup table function. */
uint8_t ucGetCos( int32_t val );

/* Create a wave pattern. */
void vLaser( const uint32_t ulPatternCount );

/* Simulate the aurora borealis. */
void vAuroraBorealis( const uint32_t usPatternCount );

/* Crossfade between the rainbow colors. */
void vRainbowCrossfade( const uint32_t usPatternCount );

/* Create a fire like pattern. */
void vFireSparks( const uint32_t usPatternCount );

/* Create an RGB audio pattern. */
void vRgbAudio ( const uint32_t ulPatternCount );

/* Calcualte the crossfade operation. */
void vCrossfade( int16_t start, uint16_t len, uint8_t ramp, uint8_t R, uint8_t G,uint8_t B );

/* Get some randomly colored pixels. */
void vGetRandPix(uint8_t* ucPix, uint16_t usNumPixels);

/*-----------------------------------------------------------*/

/* Get LED buffer from other file (leds.c). */
extern volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS];

/* Get ADC buffer full from other file (audio.c). */
extern uint8_t ucAdcBufferFull;

/* Get Adc sample buffer from other file (audio.c). */
extern float32_t ufAdcSampleBuffer[ADC_SAMPLES];

/* Get rfft instance from other file (audio.c). */
extern arm_rfft_fast_instance_f32 S;

/*-----------------------------------------------------------*/

#endif