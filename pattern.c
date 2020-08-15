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
    const TickType_t xFrequency = 50;
    UBaseType_t xAvailableStack = 0;

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
            vFillStrip( 0, 0, 0 );

            eCurrentPattern = RGB_RAMP;
            usPatternCount = 0;
            break;

        case RGB_RAMP:
            /* Ramp from one color to the next. */
            vRainbowCrossfade();

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
  * @brief  Crossfades between the RGB colors.
  * @retval None
  */
void vRainbowCrossfade( void )
{
    static colors_t eColor = RED;

    switch (eColor)
    {
    case RED:
        vFillStrip( 255, 0, 0 );
        break;
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Crossfades between the RGB colors.
  * @retval None
  */
/*void vCrossfade( uint16_t start, uint16_t len, bool ramp, uint8_t R, uint8_t G,uint8_t B )
{
    uint32_t counter = 0;
    if( ramp )
    {
        for( int i = start + len; i > start; i-- )
        {
            setLED(i,
                   getLED(i, RED)* (counter)/len+R*(len-counter)/len,
                   getLED(i, GRN)* (counter)/len+G*(len-counter)/len,
                   getLED(i, BLU)* (counter)/len+B*(len-counter)/len);
            counter++;
        }
    }
    else
    {
        for( int i = start; i < ( start + len ); i++ )
        {
            setLED(i,
                   getLED(i, RED)* (counter)/len+R*(len-counter)/len,
                   getLED(i, GRN)* (counter)/len+G*(len-counter)/len,
                   getLED(i, BLU)* (counter)/len+B*(len-counter)/len);
            counter++;
        }
    }
}*/
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
void vSetLed( uint16_t LED, uint8_t R, uint8_t G, uint8_t B )
{
    ucLeds[LED][GRN] = G;
    ucLeds[LED][RED] = R;
    ucLeds[LED][BLU] = B;
}
/*-----------------------------------------------------------*/


/**
  * @brief  Gets the the color channel brightness.
  * @retval Color channel brightness.
  */
uint8_t ucGetLed( uint16_t LED, uint8_t color )
{
    return ucLeds[LED][color];
}
/*-----------------------------------------------------------*/