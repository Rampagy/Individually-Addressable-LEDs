#include "pattern.h"

/* Task Handle. */
TaskHandle_t xCreatePatternHandle = NULL;

/**
  * @brief  Task that initiates the PWM stream to the individually addressable LEDs.
  * @note   Executes every 10ms resulting in a 100Hz update rate.
  * @retval None
  */
void vCreatePattern( void * pvParameters  )
{
    UBaseType_t xAvailableStack = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;

    /* Initialize the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    while ( 1 )
    {
        /* Check the stack size. */
        xAvailableStack = uxTaskGetStackHighWaterMark( xCreatePatternHandle );

        if (xAvailableStack <= 10)
        {
            /* Turn orangle LED on if stack overflow is imminent/detected. */
            STM_EVAL_LEDOn( LED4 );
        }

        /* Wait for the next cycle. */
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}