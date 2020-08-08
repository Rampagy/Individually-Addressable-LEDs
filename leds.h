#ifndef LEDS_H
#define LEDS_H

/* Peripheral includes. */
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"

/* Defines for the SK6812 LEDs. */
#define NUMBER_OF_LEDS 144

/* 106 cycles * (1 sec / 84,000,000 cycle ) = 1.26 usec */
#define CLOCK_THRESH 106

/* 25 cycles * (1 sec / 84,000,000 cycles) = 0.298 usec */
#define LOW_THRESH 25

/* 50 cycles * (1 sec / 84,000,000 cycles) = 0.595 usec*/
#define HIGH_THRESH 50

/* 6720 cycles * (1 sec / 84,000,000 cycles) = 80.0 usec */
#define RESET_CYCLES 6720

/* RESET_CYCLES / CLOCK_THRESH = 64 */
#define RESET_PERIODS 64

/* Number of PWM periods to send a whole LED strip.
Number of LEDs * (Color Channels / LED) * (Bits / Color Channel) */
#define LED_PERIODS (NUMBER_OF_LEDS * 3 * 8)

/* Total nuumber of periods to send the entire LED strip */
#define TOTAL_PERIODS (LED_PERIODS + RESET_PERIODS)

/* Function prototypes. */
void vInitLeds(void);

#endif