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
    TickType_t xLastWakeTime;
    UBaseType_t xAvailableStack = 0;

    /* Initialize the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    /* Local variable for calculating the pattern. */
    patterns_t eCurrentPattern = OFF;
    uint32_t usPatternCount = 0;

    while ( 1 )
    {
        switch ( eCurrentPattern )
        {
        default:
        case OFF:
            /* Turn LEDs off and immediately go to the next pattern. */
            vFillStrip( 0, 0, 0 );

            eCurrentPattern = RGB_RAMP;
            usPatternCount = 0;
            break;

        case RGB_RAMP:
            /* Ramp from one color to the next. */
            vRainbowCrossfade();

            usPatternCount += configPATTERN_TASK_TIME_MS;
            if ( usPatternCount >= configRAINBOW_CROSSFADE_TIME_MS )
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
        vTaskDelayUntil( &xLastWakeTime, configPATTERN_TASK_TIME_MS );
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Crossfades between the RGB colors.
  * @retval None
  */
void vRainbowCrossfade( void )
{
    static colors_t eColor = RED;
    static uint16_t usRainbowCount = 0;
    static int16_t usLedCount = -configRAINBOW_TRANSITION_LENGTH;

    /* Increment the task count. */
    usRainbowCount += configPATTERN_TASK_TIME_MS;

    switch (eColor)
    {
    case RED:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( usRainbowCount >= configRAINBOW_RAMP_TIME_MS ) )
        {
            usRainbowCount = 0;
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 255, 0, 0 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = GRN;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
            usRainbowCount = 0;
        }
        break;

    case GRN:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( usRainbowCount >= configRAINBOW_RAMP_TIME_MS ) )
        {
            usRainbowCount = 0;
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 0, 255, 0 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = BLU;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
            usRainbowCount = 0;
        }
        break;

    case BLU:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( usRainbowCount >= configRAINBOW_RAMP_TIME_MS ) )
        {
            usRainbowCount = 0;
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 0, 0, 255 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = RED;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
            usRainbowCount = 0;
        }
        break;
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Crossfades between the RGB colors.
  * @note   If ramp then 0 to 255, else 255 to 0
  * @retval None
  */
void vCrossfade( int16_t start, uint16_t len, uint8_t ramp, uint8_t R, uint8_t G,uint8_t B )
{
    uint32_t counter = 0;
    if( ramp )
    {
        /* Iterate from the end to the start. */
        for( int16_t i = start + len; i > start; i-- )
        {
            vSetLed(i,
                   ucGetLed(i, RED) * counter / len + R * ( len - counter ) / len,
                   ucGetLed(i, GRN) * counter / len + G * ( len - counter ) / len,
                   ucGetLed(i, BLU) * counter / len + B * ( len - counter ) / len);
            counter++;
        }
    }
    else
    {
        for( int16_t i = start; i < ( start + len ); i++ )
        {
            vSetLed(i,
                   ucGetLed(i, RED) * counter / len + R * ( len - counter ) / len,
                   ucGetLed(i, GRN) * counter / len + G * ( len - counter ) / len,
                   ucGetLed(i, BLU) * counter / len + B * ( len - counter ) / len);
            counter++;
        }
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Fill the entire strip with color.
  * @retval None
  */
void vFillStrip( uint8_t R, uint8_t G, uint8_t B )
{
    for( uint16_t i = 0; i < NUMBER_OF_LEDS; i++ )
    {
        vSetLed(i, R, G, B);
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Sets the RGB value for an led.
  * @retval None
  */
void vSetLed( int16_t LED, uint8_t R, uint8_t G, uint8_t B )
{
    if ( ( LED < NUMBER_OF_LEDS ) && ( LED >= 0 ) )
    {
        ucLeds[LED][GRN] = G;
        ucLeds[LED][RED] = R;
        ucLeds[LED][BLU] = B;
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Gets the the color channel brightness.
  * @retval Color channel brightness.
  */
uint8_t ucGetLed( int16_t LED, uint8_t color )
{
    uint8_t ucLedColor = 0;
    if ( ( LED < NUMBER_OF_LEDS ) && ( LED >= 0 ) && ( color <= BLU ) )
    {
        ucLedColor = ucLeds[LED][color];
    }

    return ucLedColor;
}
/*-----------------------------------------------------------*/