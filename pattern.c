#include "pattern.h"

/* Task handle. */
TaskHandle_t xCreatePatternHandle = NULL;

/**
  * @brief  Task chooses pattern and calculates the colors.
  * @note   Executes every 50ms.
        Might increase/decrease the rate depending on CPU usage.
  * @retval None
  */
void vCreatePattern( void * pvParameters  )
{
    UBaseType_t xAvailableStack = 0;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 50;

    /* Initialize the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    /* Local variable for calculating the pattern. */
    patterns_t eCurrentPattern = OFF;
    uint16_t usPatternCount = 0;

    while ( 1 )
    {
        switch ( eCurrentPattern )
        {
        default:
        case OFF:
            /* Turn LEDs off and immediately go to the next pattern. */
            vTurnLedsOff();

            eCurrentPattern = RGB_RAMP;
            usPatternCount = 0;
            break;

        case RGB_RAMP:
            usPatternCount += portTICK_PERIOD_MS;

            if ( usPatternCount >= configRGB_RAMP_TIME_MS )
            {
                /* Switch to the new pattern and reset the counter */
                eCurrentPattern = RGB_RAMP;
                usPatternCount = 0;
            }
            break;
        }

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
/*-----------------------------------------------------------*/


/**
  * @brief  Turns all LEDS off.
  * @retval None
  */
void vTurnLedsOff( void )
{
    for (uint16_t usI = 0; usI < NUMBER_OF_LEDS; usI++)
    {
        for (uint8_t ucJ = 0; ucJ < COLOR_CHANNELS; ucJ++)
        {
            ucLeds[usI][ucJ] = 0;
        }
    }
}