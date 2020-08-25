#include "pattern.h"

/* Task handle. */
TaskHandle_t xCreatePatternHandle = NULL;

/* Cosine lookup table. */
uint8_t cos255[360] = {
    255,255,255,255,255,254,254,254,253,253,253,252,252,251,251,250,249,249,248,
    247,247,246,245,244,243,242,241,240,239,238,237,236,234,233,232,231,229,228,
    227,225,224,222,221,219,218,216,214,213,211,209,208,206,204,202,201,199,197,
    195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,160,158,
    156,154,152,150,147,145,143,141,139,136,134,132,130,128,125,123,121,119,116,
    114,112,110,108,105,103,101,99,97,95,92,90,88,86,84,82,80,78,76,74,72,70,68,
    66,64,62,60,58,56,54,53,51,49,47,46,44,42,41,39,37,36,34,33,31,30,28,27,26,
    24,23,22,21,19,18,17,16,15,14,13,12,11,10,9,8,8,7,6,6,5,4,4,3,3,2,2,2,1,1,1,
    0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,2,3,3,4,4,5,6,6,7,8,8,9,10,11,12,13,14,15,
    16,17,18,19,21,22,23,24,26,27,28,30,31,33,34,36,37,39,41,42,44,46,47,49,51,
    53,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,95,97,99,101,
    103,105,108,110,112,114,116,119,121,123,125,128,130,132,134,136,139,141,143,
    145,147,150,152,154,156,158,160,163,165,167,169,171,173,175,177,179,181,183,
    185,187,189,191,193,195,197,199,201,202,204,206,208,209,211,213,214,216,218,
    219,221,222,224,225,227,228,229,231,232,233,234,236,237,238,239,240,241,242,
    243,244,245,246,247,247,248,249,249,250,251,251,252,252,253,253,253,254,254,
    254,255,255,255,255,255,255
};

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
    patterns_t eCurrentPattern = WAVE;
    uint32_t ulPatternCount = 0;

    while ( 1 )
    {
        switch ( eCurrentPattern )
        {
        default:
        case OFF:
            /* Turn LEDs off and immediately go to the next pattern. */
            vFillStrip( 0, 0, 0 );

            eCurrentPattern = RGB_RAMP;
            ulPatternCount = 0;
            break;

        case RGB_RAMP:
            /* Ramp from one color to the next. */
            vRainbowCrossfade( ulPatternCount );

            ulPatternCount += configPATTERN_TASK_TIME_MS;
            if ( ulPatternCount >= configRAINBOW_CROSSFADE_TIME_MS )
            {
                /* Switch to the new pattern and reset the counter */
                eCurrentPattern = AURORA_BOREALIS;
                ulPatternCount = 0;
            }
            break;

        case AURORA_BOREALIS:
            /* Visualize the northern lights. */
            vAuroraBorealis( ulPatternCount );

            ulPatternCount += configPATTERN_TASK_TIME_MS;
            if ( ulPatternCount >= configAURORA_BOREALIS_TIME_MS )
            {
                /* Switch to new pattern and reset the counter. */
                eCurrentPattern = WAVE;
                ulPatternCount = 0;
            }
            break;

        case WAVE:
            /* Create a wave pattern. */
            vWave( ulPatternCount );

            ulPatternCount += configPATTERN_TASK_TIME_MS;
            if ( ulPatternCount >= configWAVE_TIME_MS )
            {
                /* Switch to new pattern and reset the counter. */
                eCurrentPattern = RGB_RAMP;
                ulPatternCount = 0;
            }
            break;
        }

        /* Check the stack size. */
        xAvailableStack = uxTaskGetStackHighWaterMark( xCreatePatternHandle );

        if (xAvailableStack <= 20)
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
  * @brief  Create wave pattern across LED strip.
  * @retval None
  */
void vWave( const uint32_t ulPatternCount )
{
    static int16_t sWaveStartLed = 0;
    static uint8_t ucColorIndex = 0;

    /* Set colors for the wave pattern. */
    static uint8_t ucColors[configWAVE_NUM_COLORS][3] = {
        configWAVE_COLORS
    };


    if ( ulPatternCount == 0 )
    {
        sWaveStartLed = -configWAVE_LENGTH;
        ucColorIndex = ulGetRandVal() % configWAVE_NUM_COLORS;
    }

    vFillStrip( 0, 0, 0 );

    for ( uint16_t i = 0; i < configWAVE_LENGTH; i++ )
    {
        /* Generate a cosine wave that starts and stops at intensity 0. */
        uint8_t sWaveIntensity = ucGetCos( 180 + ( i * ( 180 / configWAVE_LENGTH ) ) );

        uint8_t R = ucColors[ucColorIndex][0] * sWaveIntensity / 255;
        uint8_t G = ucColors[ucColorIndex][1] * sWaveIntensity / 255;
        uint8_t B = ucColors[ucColorIndex][2] * sWaveIntensity / 255;

        vSetLed( sWaveStartLed + i, R, G, B );
    }

    sWaveStartLed++;

    /* If the wave goes off the end of the strip. */
    if ( sWaveStartLed > NUMBER_OF_LEDS + configWAVE_LENGTH )
    {
        sWaveStartLed = -configWAVE_LENGTH;
        ucColorIndex = ulGetRandVal() % configWAVE_NUM_COLORS;
    }
}
/*-----------------------------------------------------------*/



/**
  * @brief  Simulate the Aurora Borealis.
  * @retval None
  */
void vAuroraBorealis( const uint32_t ulPatternCount )
{
    /* Number of aurora's simultaneously show. */
    static int16_t sAuroraArray[configAURORA_BOREALIS_LENGTH];

    /* Aurora color.*/
    static uint16_t usAuroraColors[configAURORA_BOREALIS_LENGTH];

    uint16_t usWidth = 20;
    uint16_t usColorOffset = 20;
    uint16_t usColorRange = 300;

    /* Initial run. */
    if ( ulPatternCount == 0 )
    {
        for ( uint16_t i = 0; i < configAURORA_BOREALIS_LENGTH; i++ )
        {
            /* Pick aurora LEDs. Biased to the end of the strip because this is
            the last LED in the aurora. */
            sAuroraArray[i] = ( ulGetRandVal() % (2 * NUMBER_OF_LEDS) ) - (NUMBER_OF_LEDS / 4);

            /* Get some random colors. */
            usAuroraColors[i] = ulGetRandVal() % usColorRange;
        }
    }

    if ( ulPatternCount % configAURORA_BOREALIS_DELAY_MS == 0 )
    {
        vFillStrip( 0, 0, 63 );

        for ( uint16_t i = 0; i < configAURORA_BOREALIS_LENGTH; i++ )
        {
            /* Blend LED colors (Green/Blue) via cosine function. */
            /* LEDs in the center are brighter than the edges. */

            /* These LEDs get brighter towards the end of the strip. */
            vCrossfade(sAuroraArray[i] - 2 * usWidth, usWidth, 1,
                       0,                                                           // R
                       ucGetCos( usAuroraColors[i] + usColorOffset ),               // G
                       ucGetCos( usAuroraColors[i] + 120 + usColorOffset ) / 2 );   // B

            /* These LEDs get dimmer towards the end of the strip. */
            vCrossfade(sAuroraArray[i] - usWidth, usWidth, 0,
                       0,                                                           // R
                       ucGetCos( usAuroraColors[i] + usColorOffset ),               // G
                       ucGetCos( usAuroraColors[i] + 120 + usColorOffset ) / 2 );   // B

            if ( i < ( configAURORA_BOREALIS_LENGTH / 2 ) )
            {
                /* Move the aurora LED towards the end of the LED strip. */
                sAuroraArray[i]++;

                /* If we increment off end of the LED strip. */
                if ( sAuroraArray[i] > ( NUMBER_OF_LEDS + 2 * usWidth ) )
                {
                    /* We know this aurora will work it's way towards the end of
                    the strip so bias the selection towards the start of the LED
                    strip. Pick a new aurora start LED in the first 33%.*/
                    sAuroraArray[i] = ulGetRandVal() % ( NUMBER_OF_LEDS / 3 ) - usWidth;
                    usAuroraColors[i] = ulGetRandVal() % usColorRange;
                }
            }
            else
            {
                /* Move the aurora LED towards the start of the LED strip. */
                sAuroraArray[i]--;

                /* If we decrement off the beginning of the LED strip. */
                if ( sAuroraArray[i] < 0 )
                {
                    /* We know this aurora will work it's way towards the start
                    of the strip so bias the selection towards the end of the LED
                    strip. Pick a new aurora start LED in the last 33%.*/
                    sAuroraArray[i] = ulGetRandVal() % ( NUMBER_OF_LEDS / 3 ) +
                                        ( 2 * NUMBER_OF_LEDS / 3 ) + usWidth;
                    usAuroraColors[i] = ulGetRandVal() % usColorRange;
                }
            }
        }
    }
}

/*-----------------------------------------------------------*/

/**
  * @brief  Crossfades between the RGB colors.
  * @retval None
  */
void vRainbowCrossfade( const uint32_t ulPatternCount )
{
    static colors_t eColor = RED;
    static int16_t usLedCount = -configRAINBOW_TRANSITION_LENGTH;

    /* Initial run. */
    if ( ulPatternCount == 0 )
    {
        eColor = (colors_t)( ulGetRandVal() % 3 );
        usLedCount = -configRAINBOW_TRANSITION_LENGTH;
    }

    switch (eColor)
    {
    case RED:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( ulPatternCount % configRAINBOW_RAMP_TIME_MS == 0 ) )
        {
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 255, 0, 0 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = GRN;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
        }
        break;

    case GRN:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( ulPatternCount % configRAINBOW_RAMP_TIME_MS == 0 ) )
        {
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 0, 255, 0 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = BLU;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
        }
        break;

    case BLU:
        if ( ( usLedCount < NUMBER_OF_LEDS ) && ( ulPatternCount % configRAINBOW_RAMP_TIME_MS == 0 ) )
        {
            usLedCount++;
            vCrossfade( usLedCount, configRAINBOW_TRANSITION_LENGTH, 0, 0, 0, 255 );
        }
        else if ( usLedCount >= NUMBER_OF_LEDS )
        {
            eColor = RED;
            usLedCount = -configRAINBOW_TRANSITION_LENGTH;
        }
        break;
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Crossfades between the RGB colors.
  * @note
        If ramp then go from end of strip to start of strip
        Else then go from start of strip to end of strip
  * @retval None
  */
void vCrossfade( int16_t start, uint16_t len, uint8_t ramp, uint8_t R, uint8_t G, uint8_t B )
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
        /* Iterate from the start to the end. */
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


/**
  * @brief  Does a cosine lookup.
  * @retval Cosine of value.
  */
uint8_t ucGetCos( int32_t val )
{
    if ( val >=  360 )
    {
        val -= ( val / 360 ) * 360;
    }

    if ( val < 0 )
    {
        val += ( val / 360 ) * 360;
    }

    if ( val == 360 )
    {
        val = 0;
    }

    return cos255[val];
}
/*-----------------------------------------------------------*/