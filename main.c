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

/******************************************************************************
 * >>>>>> NOTE 1: <<<<<<
 *
 * main() can be configured to create either a very simple LED flasher demo, or
 * a more comprehensive test/demo application.
 *
 * To create a very simple LED flasher example, set the
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY constant (defined below) to 1.  When
 * this is done, only the standard demo flash tasks are created.  The standard
 * demo flash example creates three tasks, each of which toggle an LED at a
 * fixed but different frequency.
 *
 * To create a more comprehensive test and demo application, set
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 0.
 *
 * >>>>>> NOTE 2: <<<<<<
 *
 * In addition to the normal set of standard demo tasks, the comprehensive test
 * makes heavy use of the floating point unit, and forces floating point
 * instructions to be used from interrupts that nest three deep.  The nesting
 * starts from the tick hook function, resulting is an abnormally long context
 * switch time.  This is done purely to stress test the FPU context switching
 * implementation, and that part of the test can be removed by setting
 * configUSE_TICK_HOOK to 0 in FreeRTOSConfig.h.
 ******************************************************************************
 *
 * main() creates all the demo application tasks and software timers, then starts
 * the scheduler.  The web documentation provides more details of the standard
 * demo application tasks, which provide no particular functionality, but do
 * provide a good example of how to use the FreeRTOS API.
 *
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Reg test" tasks - These fill both the core and floating point registers with
 * known values, then check that each register maintains its expected value for
 * the lifetime of the task.  Each task uses a different set of values.  The reg
 * test tasks execute with a very low priority, so get preempted very
 * frequently.  A register containing an unexpected value is indicative of an
 * error in the context switching mechanism.
 *
 * "Check" timer - The check software timer period is initially set to three
 * seconds.  The callback function associated with the check software timer
 * checks that all the standard demo tasks, and the register check tasks, are
 * not only still executing, but are executing without reporting any errors.  If
 * the check software timer discovers that a task has either stalled, or
 * reported an error, then it changes its own execution period from the initial
 * three seconds, to just 200ms.  The check software timer callback function
 * also toggles an LED each time it is called.  This provides a visual
 * indication of the system status:  If the LED toggles every three seconds,
 * then no issues have been discovered.  If the LED toggles every 200ms, then
 * an issue has been discovered with at least one task.
 *
 * Tick hook - The application tick hook is called from the schedulers tick
 * interrupt service routine when configUSE_TICK_HOOK is set to 1 in
 * FreeRTOSConfig.h.  In this example, the tick hook is used to test the kernels
 * handling of the floating point units (FPU) context, both at the task level
 * and when nesting interrupts access the floating point unit registers.  The
 * tick hook function first fills the FPU registers with a known value, it
 * then triggers a medium priority interrupt.  The medium priority interrupt
 * fills the FPU registers with a different value, and triggers a high priority
 * interrupt.  The high priority interrupt once again fills the the FPU
 * registers with a known value before returning to the medium priority
 * interrupt.  The medium priority interrupt checks that the FPU registers
 * contain the values that it wrote to them, then returns to the tick hook
 * function.  Finally, the tick hook function checks that the FPU registers
 * contain the values that it wrote to them, before it too returns.
 *
 * Button interrupt - The button marked "USER" on the starter kit is used to
 * demonstrate how to write an interrupt service routine, and how to synchronise
 * a task with an interrupt.  A task is created that blocks on a test semaphore.
 * When the USER button is pressed, the button interrupt handler gives the
 * semaphore, causing the task to unblock.  When the task unblocks, it simply
 * increments an execution count variable, then returns to block on the
 * semaphore again.
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Project Includes */
#include "partest.h"
#include "flash.h"
#include "flop.h"
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "dynamic.h"
#include "BlockQ.h"
#include "blocktim.h"
#include "countsem.h"
#include "GenQTest.h"
#include "recmutex.h"
#include "death.h"

/* Hardware and starter kit includes. */
#include "arm_comm.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1UL )

/*-----------------------------------------------------------*/

static void vLed3ToggleTask( void * pvParameters  );
static void vLed4ToggleTask( void * pvParameters  );
static void vLed5ToggleTask( void * pvParameters  );
static void vLed6ToggleTask( void * pvParameters  );

/*-----------------------------------------------------------*/

int main(void)
{
    /* Initialize all four LEDs built into the starter kit */
    STM_EVAL_LEDInit( LED3 );
    STM_EVAL_LEDInit( LED4 );
    STM_EVAL_LEDInit( LED5 );
    STM_EVAL_LEDInit( LED6 );

    /* Spawn the tasks. */
    /*           Task,                  Task Name,       Stack Size,                      parameters,         priority,                    task handle */
    xTaskCreate( vLed3ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vLed4ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vLed5ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );
    xTaskCreate( vLed6ToggleTask,       "LEDx",          configMINIMAL_STACK_SIZE,        NULL,               mainLED_TASK_PRIORITY,       ( TaskHandle_t * ) NULL );

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

static void vLed3ToggleTask( void * pvParameters  )
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 125;

    /* Initialize the xLastWakeTime variable with the current time. */
     xLastWakeTime = xTaskGetTickCount();

     /* Your code goes here */
     STM_EVAL_LEDToggle(LED3);

    while ( 1 )
    {
        // Wait for the next cycle.
         vTaskDelayUntil( &xLastWakeTime, xFrequency );

        /* Your code goes here */
        STM_EVAL_LEDToggle(LED3);
    }
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