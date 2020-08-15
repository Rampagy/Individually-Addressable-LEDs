#ifndef PATTERN_H
#define PATTERN_H

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Peripheral includes. */
#include "leds.h"

/* Patterns */
typedef enum {
    OFF,
    RGB_RAMP
} patterns_t;

/* Task for creating patterns for the LED strip. */
void vCreatePattern( void * );

#endif