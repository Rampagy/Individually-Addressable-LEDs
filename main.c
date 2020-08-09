/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Project includes. */
#include "leds.h"

/* Hardware and starter kit includes. */
#include "arm_comm.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY                   ( tskIDLE_PRIORITY + 1UL )
#define mainUPDATE_LEDS_PRIORTY                 ( tskIDLE_PRIORITY + 3UL )


/* Function prototypes. */
static void vLed3ToggleTask( void * pvParameters  );
static void vLed4ToggleTask( void * pvParameters  );
static void vLed5ToggleTask( void * pvParameters  );
static void vLed6ToggleTask( void * pvParameters  );

/* Task Handles. */
extern TaskHandle_t xUpdateLedsHandle;

/*-----------------------------------------------------------*/

int main(void)
{
    /* Initialize all four LEDs built into the starter kit */
    STM_EVAL_LEDInit( LED4 );
    STM_EVAL_LEDInit( LED5 );
    STM_EVAL_LEDInit( LED6 );

    /* Initialize the individually addressable LEDs. */
    vInitLeds();

    /* Spawn the tasks. */
    /*           Task,                  Task Name,       Stack Size,                      parameters,         priority,                    task handle */
    xTaskCreate( vLed4ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vLed5ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vLed6ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vUpdatedLedStrip,      "UpdateLeds",    configMINIMAL_STACK_SIZE,        NULL,               mainUPDATE_LEDS_PRIORTY,     &xUpdateLedsHandle );

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    for( ;; );
}
/*-----------------------------------------------------------*/

static void vLed4ToggleTask( void * pvParameters  )
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 250;

    /* Initialize the xLastWakeTime variable with the current time. */
     xLastWakeTime = xTaskGetTickCount();

     /* Your code goes here */
     STM_EVAL_LEDToggle(LED4);

    while ( 1 )
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        /* Your code goes here */
        STM_EVAL_LEDToggle(LED4);
    }
}
/*-----------------------------------------------------------*/

static void vLed5ToggleTask( void * pvParameters  )
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 500;

    /* Initialize the xLastWakeTime variable with the current time. */
     xLastWakeTime = xTaskGetTickCount();

     /* Your code goes here */
     STM_EVAL_LEDToggle(LED5);

    while ( 1 )
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );

        /* Your code goes here */
        STM_EVAL_LEDToggle(LED5);
    }
}
/*-----------------------------------------------------------*/

static void vLed6ToggleTask( void * pvParameters  )
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1000;

    /* Initialize the xLastWakeTime variable with the current time. */
     xLastWakeTime = xTaskGetTickCount();

     /* Your code goes here */
     STM_EVAL_LEDToggle(LED6);

    while ( 1 )
    {
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );


        /* Your code goes here */
        STM_EVAL_LEDToggle(LED6);
    }
}
/*-----------------------------------------------------------*/