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
#include "system_stm32f4xx.h"
#include "leds.h"
#include "pattern.h"
#include "audio.h"

/* Hardware and starter kit includes. */
#include "arm_comm.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/* Priorities for the demo application tasks (higher numbers preempt lower numbers) */
#define mainUPDATE_LEDS_PRIORITY                ( tskIDLE_PRIORITY + 2UL )
#define mainCREATE_PATTERN_PRIORITY             ( tskIDLE_PRIORITY + 1UL )

/* Task stack sizes. */
#define configUPDATE_LEDS_STACK_SIZE            configMINIMAL_STACK_SIZE
#define configCREATE_PATTERN_STACK_SIZE         configMINIMAL_STACK_SIZE

/* Task Handles. */
extern TaskHandle_t xUpdateLedsHandle;
extern TaskHandle_t xCreatePatternHandle;

/*-----------------------------------------------------------*/

int main(void)
{
    /* Update the MCU and peripheral clock frequencies */
    SystemCoreClockUpdate();

    /* Initialize all four LEDs built into the starter kit */
    STM_EVAL_LEDInit( LED3 );       // Update LEDs stack overflow (ORG)
    STM_EVAL_LEDInit( LED4 );       // Create Pattern stack overflow (GRN)
    STM_EVAL_LEDInit( LED5 );
    STM_EVAL_LEDInit( LED6 );

    /* Initialize the individually addressable LEDs. */
    vInitLeds();

    /* Initialize the ADC/DMA/DSP instructions. */
    vInitAudio();

    /* Spawn the tasks. */
    /*           Task,                  Task Name,          Stack Size,                             parameters,     priority,                           task handle */
    xTaskCreate( vUpdateLedStrip,       "UpdateLeds",       configUPDATE_LEDS_STACK_SIZE,           NULL,           mainUPDATE_LEDS_PRIORITY,            &xUpdateLedsHandle );
    xTaskCreate( vCreatePattern,        "CreatePattern",    configCREATE_PATTERN_STACK_SIZE,        NULL,           mainCREATE_PATTERN_PRIORITY,        &xCreatePatternHandle );

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* If all is well, the scheduler will now be running, and the following line
    will never be reached.  If the following line does execute, then there was
    insufficient FreeRTOS heap memory available for the idle and/or timer tasks
    to be created.  See the memory management section on the FreeRTOS web site
    for more details. */
    while ( 1 );
}
/*-----------------------------------------------------------*/