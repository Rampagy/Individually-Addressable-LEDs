#ifndef LEDS_H
#define LEDS_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Peripheral includes. */
#include "stm32f4_discovery.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rng.h"

/* User includes */
#include "debug.h"

/*-----------------------------------------------------------*/

/* Defines for the SK6812 LEDs. */
#define NUMBER_OF_LEDS 144

/* Number of color channels */
#define COLOR_CHANNELS 3

/* 52 cycles * (1 sec / 42,000,000 cycle ) = 1.25 usec */
#define CLOCK_THRESH ( 52 )

/* 13 cycles * (1 sec / 42,000,000 cycles) = 0.310 usec */
#define LOW_THRESH ( 13 )

/* 25 cycles * (1 sec / 42,000,000 cycles) = 0.595 usec*/
#define HIGH_THRESH ( 50 )

/* 3360 cycles * (1 sec / 42,000,000 cycles) = 80.0 usec */
#define RESET_CYCLES 3360

/* RESET_CYCLES / CLOCK_THRESH = 64 */
#define RESET_PERIODS 65

/* Number of PWM periods to send a whole LED strip.
Number of LEDs * (Color Channels / LED) * (Bits / Color Channel) */
#define LED_PERIODS (NUMBER_OF_LEDS * COLOR_CHANNELS * 8)

/* Total nuumber of periods to send the entire LED strip */
#define TOTAL_PERIODS (LED_PERIODS + RESET_PERIODS)

/*-----------------------------------------------------------*/

/* Function for initializing the hardware to talk to the LEDs. */
void vInitLeds( void );

/* Task for updating the LED strip. */
void vUpdateLedStrip( void * );

/* Function for getting a random number. */
uint32_t ulGetRandVal( void );

/*-----------------------------------------------------------*/

/* Get debug stats from other file (debug.c) */
extern xDebugStats_t xDebugStats;

#endif