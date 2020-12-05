#include "pattern.h"

/* Task handle. */
TaskHandle_t xCreatePatternHandle = NULL;

/* Available stack size. */
UBaseType_t xPatternAvailableStack = 0;

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
  * @note   Executes every 10ms.
        Might increase/decrease the rate depending on CPU usage.
  * @retval None
  */
void vCreatePattern( void * pvParameters  )
{
    TickType_t xLastWakeTime;

    /* Initialize the xLastWakeTime variable with the current time. */
    xLastWakeTime = xTaskGetTickCount();

    /* Initialize start time.*/
    uint16_t usStartTime = TIM12->CNT;

    /* Local variable for calculating the pattern. */
#if ( configALL == 1 ) || ( configNO_AUDIO == 1 )
    patterns_t eCurrentPattern = (patterns_t)0;
#elif ( configONLY_AUDIO == 1 )
    patterns_t eCurrentPattern = (patterns_t)(RGB_AUDIO); //(AUDIO_TRAIN); // ( AUDIO_PATTERNS + 1 );
#endif
    uint32_t ulPatternCount = 0;

    while ( 1 )
    {
        switch ( eCurrentPattern )
        {
        default:
        case RAINBOW_CROSSFADE:
            /* Ramp from one color to the next. */
            vRainbowCrossfade( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configRAINBOW_CROSSFADE_TIME_MS, &eCurrentPattern );
            break;

        case AURORA_BOREALIS:
            /* Visualize the northern lights. */
            vAuroraBorealis( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configAURORA_BOREALIS_TIME_MS, &eCurrentPattern );
            break;

        case LASER:
            /* Create a wave pattern. */
            vLaser( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configLASER_TIME_MS, &eCurrentPattern );
            break;

        case FIRE_SPARKS:
            /* Create a fire with sparks pattern. */
            vFireSparks ( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configFIRE_SPARKS_TIME_MS, &eCurrentPattern );
            break;

        case RGB_AUDIO:
            /* Create RGB audio pattern. */
            /** @TODO: This function call is overruinning the task rate by 1000%. Need to fix. */
            //vRgbAudio ( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configRGB_AUDIO_TIME_MS, &eCurrentPattern );
            break;

        case AUDIO_TRAIN:
            /* Create audio train pattern. */
            vAudioTrain( ulPatternCount );

            /* Check for next pattern. */
            vCheckNextPattern( &ulPatternCount, configAUDIO_TRAIN_TIME_MS, &eCurrentPattern );
        }

        /* Check the stack size. */
        xPatternAvailableStack = uxTaskGetStackHighWaterMark( xCreatePatternHandle );

        if ( xPatternAvailableStack <= 20 )
        {
            /* Turn orangle LED on if stack overflow is imminent/detected. */
            STM_EVAL_LEDOn( LED4 );
        }

        /* Update the high water mark for task length. */
        if ( (uint16_t)TIM12->CNT - usStartTime > xDebugStats.usPatternClocks )
        {
            xDebugStats.usPatternClocks = (uint16_t)TIM12->CNT - usStartTime;
        }

        /* Wait for the next cycle. */
        vTaskDelayUntil( &xLastWakeTime, configPATTERN_TASK_TIME_MS );

        /* Initialize the start time. */
        usStartTime = TIM12->CNT;
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Create audio train pattern.
  * @retval None
  */
void vAudioTrain ( const uint32_t ulPatternCount )
{
    /* Create array for storing frequency information. */
    float32_t ufComplexFFT[ADC_SAMPLES] = { 0.0F };

    /* The real FFT does not provide symmetry so divide FFT_SIZE by 2. */
    float32_t ufFourierFrequency[FFT_SIZE/2] = { 0.0F };

    /* Scaling vector. */
    //float32_t ufScalingVector[FFT_SIZE/2] = { 0.0F };

    /* FFT max variables */
    float32_t ufMaxFFTMag = 0.0F;
    float32_t ufDCOffset = 0.0F;
    uint32_t usMaxFFTIdx = 0U;

    /* DC offset has a different scaling than the non-zero frequencies. */
    //ufScalingVector[0] = 1.0 / FFT_SIZE;
    //for (uint16_t i = 1; i < (FFT_SIZE/2); i++)
    //{
    //    /* I do not know where these scalings come from, just that they work. */
    //    ufScalingVector[i] = 1.0 / (FFT_SIZE/2.0);
    //}

    /* This can be uncommented to test the fft logic. */
    //for (uint16_t i = 0; i < ADC_SAMPLES; i++)
    //{
    //    /* Frequency magnitude at index 10 (4410 Hz) should be the max at 10
    //     * with a DC offset of 50.
    //     */
    //    ufAdcSampleBuffer[i] = 10*arm_sin_f32( 2.0*3.14159F*i/8.0 ) + 50;
    //}

    /* Perform real FFT (nobody understands complex numbers anyways).
     * Enter critical here to ensure that the AdcSampleBuffer is not
     * updated in the middle of the FFT.*/
    taskENTER_CRITICAL();
    arm_rfft_fast_f32( &S, ufAdcSampleBuffer, ufComplexFFT, 0 );
    taskEXIT_CRITICAL();

    /* Save DC offset into temporary variable because arm_cmplx_mag_f32 clears it out. */
    float32_t temp = ufComplexFFT[0];

    /* Output of FFT is always complex, so convert to magnitude. */
    arm_cmplx_mag_f32( ufComplexFFT, ufFourierFrequency, FFT_SIZE );

    /* Complex magnitude is not applicable to the DC offset. */
    ufFourierFrequency[0] = temp;

    /* Rescale ufFourierFrequency to its true amplitudes.
     * I have no idea where these scalings come from, but it works. */
    //arm_mult_f32( ufFourierFrequency, ufScalingVector, ufFourierFrequency, FFT_SIZE/2 );

    /* Get the max FFT magnitude and its index. */
    ufDCOffset = ufFourierFrequency[0];
    ufFourierFrequency[0] = 0.0F;
    arm_max_f32( ufFourierFrequency, FFT_SIZE/2, &ufMaxFFTMag, &usMaxFFTIdx );

    /** @TODO: Make pattern based on frequency data.
      * @NOTE: ufFourierFrquency[0] is the dc offset.  To get the true dc offset
      *     you have to divide this number by FFT_SIZE or ADC_SAMPLES.
      * @NOTE: The rest of the values are scaled by FFT_SIZE/2 or ADC_SAMPLES/4.
      *     So to get the true magnitude of the frequencies you have to divide
      *     by FFT_SIZE/2 or ADC_SAMPLES/4.
      *
      * The frequency that corresponds to each index can be determined by:
      *     Frequency = Idx * SamplingFrequency / FFT_SIZE
      *
      * For this project it equates to ~345Hz per index.
      *     344.5 Hz = 44100 Hz / 128
      */

    /* Shift all of the LEDs one further down the strip. */
    for ( uint16_t i = NUMBER_OF_LEDS-1; i > 0; i-- )
    {
        vSetLed( i,
                (int16_t)ucGetLed( i-1, RED ),
                (int16_t)ucGetLed( i-1, GRN ),
                (int16_t)ucGetLed( i-1, BLU ) );
    }

    uint16_t usSectionFreq[configAUDIO_TRAIN_NUM_FREQUENCIES] = { 0U };
    uint16_t usMaxSectionFreq[configAUDIO_TRAIN_NUM_FREQUENCIES] = {
        configAUDIO_TRAIN_FEQUENCIES
    };

    /* Determine the LED brightnesses. */
    for ( uint16_t i = 0; i < configAUDIO_TRAIN_NUM_FREQUENCIES; i++ )
    {
        /* Scroll through each frequency band and see if it fits into any of them. */
        for ( uint16_t j = 1; j < FFT_SIZE / 2; j++ )
        {
            /* Get the frequency associated to the index. */
            uint32_t ulIdxFreq = ( (uint32_t)j * SAMPLING_FREQUENCY ) / FFT_SIZE;

            /* Find max frequency within the allocated range for the section. */
            if ( ulIdxFreq <= usMaxSectionFreq[i] )
            {
                /* Order of evaluations is important here. */
                if ( ( i == 0 ) ||
                     ( ulIdxFreq > usMaxSectionFreq[i-1] ) )
                {
                    /* Chekc if the frequency is in the band. If so, track the max for this band. */
                    if ( ufFourierFrequency[j] > usSectionFreq[i] )
                    {
                        usSectionFreq[i] = (uint16_t)ufFourierFrequency[j];
                    }
                }
            }
            else
            {
                break;
            }
        }
    }

    /* Set the first LED to its new color. */
    //int16_t R = (int16_t)( ( usSectionFreq[0] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );
    //int16_t G = (int16_t)( ( usSectionFreq[1] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );
    //int16_t B = (int16_t)( ( usSectionFreq[2] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );

    if ( ufFourierFrequency[2] > configAUDIO_TRAIN_MAX_BRIGHTNESS ) ufFourierFrequency[2] = configAUDIO_TRAIN_MAX_BRIGHTNESS;
    if ( ufFourierFrequency[15] > configAUDIO_TRAIN_MAX_BRIGHTNESS ) ufFourierFrequency[15] = configAUDIO_TRAIN_MAX_BRIGHTNESS;
    if ( ufFourierFrequency[35] > configAUDIO_TRAIN_MAX_BRIGHTNESS ) ufFourierFrequency[35] = configAUDIO_TRAIN_MAX_BRIGHTNESS;

    int16_t R = (int16_t)( ( ufFourierFrequency[2] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );
    int16_t G = (int16_t)( ( ufFourierFrequency[15] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );
    int16_t B = (int16_t)( ( ufFourierFrequency[35] * 255 ) / configAUDIO_TRAIN_MAX_BRIGHTNESS );

    vSetLed( 0, R, G, B );
}
/*-----------------------------------------------------------*/



/**
  * @brief  Create RGB audio pattern.
  * @retval None
  */
void vRgbAudio ( const uint32_t ulPatternCount )
{
    /* Create array for storing frequency information. */
    float32_t ufComplexFFT[ADC_SAMPLES] = { 0.0F };

    /* The real FFT does not provide symmetry so divide FFT_SIZE by 2. */
    float32_t ufFourierFrequency[FFT_SIZE/2] = { 0.0F };

    /* Scaling vector.
     * This can be uncommented to normalize back to amplitude. */
    //float32_t ufScalingVector[FFT_SIZE/2] = { 0.0F };

    /* FFT max variables */
    float32_t ufMaxFFTMag = 0.0F;
    float32_t ufDCOffset = 0.0F;
    uint32_t usMaxFFTIdx = 0U;

    /* DC offset has a different scaling than the non-zero frequencies.
     * This can be uncommented to normalize back to amplitude. */
    //ufScalingVector[0] = 1.0 / FFT_SIZE;
    //for (uint16_t i = 1; i < (FFT_SIZE/2); i++)
    //{
    //    /* I do not know where these scalings come from, just that they work. */
    //    ufScalingVector[i] = 1.0 / (FFT_SIZE/2.0);
    //}

    /* This can be uncommented to test the fft logic. */
    //for (uint16_t i = 0; i < ADC_SAMPLES; i++)
    //{
    //    /* Frequency magnitude at index 10 (4410 Hz) should be the max at 10
    //     * with a DC offset of 50.
    //     */
    //    ufAdcSampleBuffer[i] = 10*arm_sin_f32( 2.0*3.14159F*i/8.0 ) + 50;
    //}

    /* Perform real FFT (nobody understands complex numbers anyways).
     * Enter critical here to ensure that the AdcSampleBuffer is not
     * updated in the middle of the FFT.*/
    taskENTER_CRITICAL();
    arm_rfft_fast_f32( &S, ufAdcSampleBuffer, ufComplexFFT, 0 );
    taskEXIT_CRITICAL();

    /* Save DC offset into temporary variable because arm_cmplx_mag_f32 clears it out. */
    float32_t temp = ufComplexFFT[0];

    /* Output of FFT is always complex, so convert to magnitude. */
    arm_cmplx_mag_f32( ufComplexFFT, ufFourierFrequency, FFT_SIZE );

    /* Complex magnitude is not applicable to the DC offset. */
    ufFourierFrequency[0] = temp;

    /* Rescale ufFourierFrequency to its true amplitudes.
     * I have no idea where these scalings come from, but it works.
     * This can be uncommented to normalize back to amplitude. */
    //arm_mult_f32( ufFourierFrequency, ufScalingVector, ufFourierFrequency, FFT_SIZE/2 );

    /* Get the max FFT magnitude and its index. */
    ufDCOffset = ufFourierFrequency[0];
    ufFourierFrequency[0] = 0.0F;
    arm_max_f32( ufFourierFrequency, FFT_SIZE/2, &ufMaxFFTMag, &usMaxFFTIdx );

    /** @TODO: Make pattern based on frequency data.
      * @NOTE: ufFourierFrquency[0] is the dc offset.  To get the true dc offset
      *     you have to divide this number by FFT_SIZE or ADC_SAMPLES.
      * @NOTE: The rest of the values are scaled by FFT_SIZE/2 or ADC_SAMPLES/4.
      *     So to get the true magnitude of the frequencies you have to divide
      *     by FFT_SIZE/2 or ADC_SAMPLES/4.
      *
      * The frequency that corresponds to each index can be determined by:
      *     Frequency = Idx * SamplingFrequency / FFT_SIZE
      *
      * For this project it equates to ~345Hz per index.
      *     344.5 Hz = 44100 Hz / 128
      */

    /* General Peak -> 10627
     * DC offset peak -> 2048047
     */

    float32_t ufSectionAverages[configRGB_AUDIO_SECTIONS] = { 0 };
    uint16_t usMaxSectionFreq[configRGB_AUDIO_SECTIONS] = {
        configRGB_AUDIO_SECTION_COLORS
    };

    /* Find the average magnitude for each frequency section. */
    for ( uint16_t i = 0; i < configRGB_AUDIO_SECTIONS; i++ )
    {
        float32_t ufAveFFTSectionMag = 0.0F;
        uint16_t usNumSectionFrequencies = 0U;

        /* Find average frequency within the allocated range for the section. */
        for ( uint16_t j = 1; j < FFT_SIZE/2; j++ )
        {
            uint32_t ulIdxFreq = ( (uint32_t)j * SAMPLING_FREQUENCY ) / FFT_SIZE;
            if ( ulIdxFreq <= usMaxSectionFreq[i] )
            {
                /* Order of evaluations is important here. */
                if ( ( i == 0 ) ||
                     ( ulIdxFreq > usMaxSectionFreq[i-1] ) )
                {
                    ufAveFFTSectionMag += (uint16_t)ufFourierFrequency[j];
                    usNumSectionFrequencies++;
                }
            }
            else
            {
                /* If we are above the section limit then there's no
                 * point in continuing the search, because it can only get bigger. */
                break;
            }
        }

        /* Calculate the average for the section. */
        ufSectionAverages[i] = ufAveFFTSectionMag / (float32_t)usNumSectionFrequencies;

        /* Saturate the averages. */
        if ( ufSectionAverages[i] > configRGB_AUDIO_MAX_BRIGHTNESS )
        {
            ufSectionAverages[i] = configRGB_AUDIO_MAX_BRIGHTNESS;
        }

        /* Now calculate and print the color for each section. */
        for (uint16_t j = 0; j < configRGB_AUDIO_LEDS_PER_SECTION; j++)
        {
            /* Check the stack size. */
            xPatternAvailableStack = uxTaskGetStackHighWaterMark( xCreatePatternHandle );

            /* Determine the LED to set. */
            uint16_t usLedIdx = i * configRGB_AUDIO_LEDS_PER_SECTION + j;

            /* Determine the brightness of the LED. */
            uint16_t usLEDBrightness = ( uint16_t )( ufSectionAverages[i] * 255 / configRGB_AUDIO_MAX_BRIGHTNESS );

            /* Set the LED color. */
            switch ( i % COLOR_CHANNELS )
            {
            default:
            case 0:
                /* Make this section RED. */
                vSetLed( usLedIdx, usLEDBrightness, 0, 0 );
                break;
            case 1:
                /* Make this section GREEN. */
                vSetLed( usLedIdx, 0, usLEDBrightness, 0 );
                break;
            case 2:
                /* Make this section BLUE. */
                vSetLed( usLedIdx, 0, 0, usLEDBrightness );
                break;
            }
        }
    }
}
/*-----------------------------------------------------------*/



/**
  * @brief  Create fire/sparks pattern.
  * @retval None
  */
void vFireSparks ( const uint32_t ulPatternCount )
{
    /* Set colors for the wave pattern. */
    static uint8_t ucColors[ configFIRE_NUM_COLORS ][ COLOR_CHANNELS ] = {
        configFIRE_COLORS
    };

    if ( ulPatternCount == 0 )
    {
        vFillStrip( 0xE2, 0x58, 0x22 );
    }

    /* Dim the strip every interval. */
    if ( ulPatternCount % configFIRE_DIM_INTERVAL == 0 )
    {
        for ( uint16_t i = 0; i < NUMBER_OF_LEDS; i++ )
        {
            vSetLed( i,
                    configFIRE_DIM_SPEED * (int16_t) ucGetLed( i, RED ) / 100,      // R
                    configFIRE_DIM_SPEED * (int16_t) ucGetLed( i, GRN ) / 100,      // G
                    configFIRE_DIM_SPEED * (int16_t) ucGetLed( i, BLU ) / 100);     // B
        }
    }

    /* Add new sparks. */
    for ( uint16_t i = 0; i < configFIRE_SPARKS; i++ )
    {
        if ( ulPatternCount % configFIRE_EMBER_INTERVAL == 0 )
        {
            uint16_t usSparkLed = ulGetRandVal() % NUMBER_OF_LEDS;
            uint16_t usLedColor = ulGetRandVal() % configFIRE_NUM_COLORS;

            vCrossfade( usSparkLed, configFIRE_EMBER_LENGTH, 0,
                        ucColors[ usLedColor ][ 0 ],      // R
                        ucColors[ usLedColor ][ 1 ],      // G
                        ucColors[ usLedColor ][ 2 ] );    // B

            vCrossfade( usSparkLed, configFIRE_EMBER_LENGTH, 1,
                        ucColors[ usLedColor ][ 0 ],      // R
                        ucColors[ usLedColor ][ 1 ],      // G
                        ucColors[ usLedColor ][ 2 ] );    // B
        }
    }
}
/*-----------------------------------------------------------*/



/**
  * @brief  Check for the next pattern.
  * @retval None
  */
void vCheckNextPattern( uint32_t* ulPatternCount, const uint32_t ulPatternLength, patterns_t* eCurrentPattern )
{
    *ulPatternCount += configPATTERN_TASK_TIME_MS;
    if ( *ulPatternCount >= ulPatternLength )
    {
        /* Switch to new pattern and reset the counter. */
        *eCurrentPattern = ( patterns_t )( *eCurrentPattern + 1 );
        if ( *eCurrentPattern == AUDIO_PATTERNS )
        {
            *eCurrentPattern = ( patterns_t )( *eCurrentPattern + 1 );
        }

#if ( configALL == 1 )
        *eCurrentPattern = ( patterns_t )( *eCurrentPattern % LAST_PATTERN );
#elif ( configNO_AUDIO == 1 )
        *eCurrentPattern = ( patterns_t )( *eCurrentPattern % AUDIO_PATTERNS );
#elif ( configONLY_AUDIO == 1 )
        *eCurrentPattern = ( patterns_t )( *eCurrentPattern % LAST_PATTERN );
        if ( *eCurrentPattern == 0 )
        {
            *eCurrentPattern = ( patterns_t )( AUDIO_PATTERNS + 1 );
        }
#endif

        *ulPatternCount = 0;
    }
}
/*-----------------------------------------------------------*/



/**
  * @brief  Create wave pattern across LED strip.
  * @retval None
  */
void vLaser( const uint32_t ulPatternCount )
{
    static int16_t sLaserStartLed = 0;
    static uint8_t ucColorIndex = 0;
    static uint8_t usIncrement = 1;

    /* Set colors for the wave pattern. */
    static uint8_t ucColors[ configLASER_NUM_COLORS ][ COLOR_CHANNELS ] = {
        configLASER_COLORS
    };


    if ( ulPatternCount == 0 )
    {
        sLaserStartLed = -configLASER_LENGTH;
        ucColorIndex = ulGetRandVal() % configLASER_NUM_COLORS;
        usIncrement = 0;
    }

    vFillStrip( 0, 0, 0 );

    for ( uint16_t i = 0; i < configLASER_LENGTH; i++ )
    {
        uint8_t sIntensity = ( ( i + 1 ) * 255 ) / configLASER_LENGTH;

        uint8_t R = ucColors[ucColorIndex][0] * sIntensity / 255;
        uint8_t G = ucColors[ucColorIndex][1] * sIntensity / 255;
        uint8_t B = ucColors[ucColorIndex][2] * sIntensity / 255;

        vSetLed( sLaserStartLed + i, R, G, B );
    }

    sLaserStartLed += usIncrement;

    if ( sLaserStartLed % configLASER_INCREMENT_DELAY == 0 )
    {
        usIncrement++;
    }

    /* If the laser goes off the end of the strip. */
    if ( sLaserStartLed > NUMBER_OF_LEDS + configLASER_LENGTH )
    {
        sLaserStartLed = -configLASER_LENGTH;
        usIncrement = 0;
        ucColorIndex = ulGetRandVal() % configLASER_NUM_COLORS;
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
    static uint8_t usColors[ configRAINBOW_SIMULTANEOUS_COLORS ][ COLOR_CHANNELS ] = { { 0 } };
    static int16_t sLedCount[ configRAINBOW_SIMULTANEOUS_COLORS ] = { 0 };

    /* Initial run. */
    if ( ulPatternCount == 0 )
    {
        vGetRandPix( &usColors[0][0], configRAINBOW_SIMULTANEOUS_COLORS );

        /* Evenly space the start positions throughout the LED strip. */
        for ( uint8_t i = 0; i < configRAINBOW_SIMULTANEOUS_COLORS; i++ )
        {
            sLedCount[i] = -configRAINBOW_TRANSITION_LENGTH + i * ( ( configRAINBOW_TRANSITION_LENGTH + NUMBER_OF_LEDS ) / configRAINBOW_SIMULTANEOUS_COLORS );
        }
    }

    for ( uint8_t i = 0; i < configRAINBOW_SIMULTANEOUS_COLORS; i++ )
    {
        if ( ( sLedCount[i] < NUMBER_OF_LEDS ) && ( ulPatternCount % configRAINBOW_RAMP_TIME_MS == 0 ) )
        {
            sLedCount[i]++;
            vCrossfade( sLedCount[i], configRAINBOW_TRANSITION_LENGTH, 0, usColors[i][0], usColors[i][1], usColors[i][2] );
        }
        else if ( sLedCount[i] >= NUMBER_OF_LEDS )
        {
            sLedCount[i] = -configRAINBOW_TRANSITION_LENGTH;
            vGetRandPix( &usColors[i][0], 1 );
        }
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Gets a random pixel.
  * @retval None
  */
void vGetRandPix(uint8_t* ucPix, uint16_t usNumPixels)
{
    for (uint16_t p = 0; p < usNumPixels; p++)
    {
        *(ucPix + ( p * COLOR_CHANNELS ) + 0) =  ulGetRandVal() % 256;  // R
        *(ucPix + ( p * COLOR_CHANNELS ) + 1) =  ulGetRandVal() % 256;  // G
        *(ucPix + ( p * COLOR_CHANNELS ) + 2) =  ulGetRandVal() % 256;  // B

        uint8_t ucRandColorChannel = ulGetRandVal() % COLOR_CHANNELS;
        uint8_t ucDifferentRandColorChannel = ( ucRandColorChannel + ( ulGetRandVal() % 2 ) + 1 ) % COLOR_CHANNELS;

        /* Randomly make one channel less than 85. */
        *(ucPix + ( p * COLOR_CHANNELS ) + ucRandColorChannel ) = ulGetRandVal() % 85;

        /* Randomly make a different channel greater than 170 .*/
        *(ucPix + ( p * COLOR_CHANNELS ) + ucDifferentRandColorChannel ) = ( ulGetRandVal() % 85 ) + 171;
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
        for( int16_t i = start; i > start - len; i-- )
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
void vSetLed( int16_t LED, int16_t R, int16_t G, int16_t B )
{
    if ( ( LED < NUMBER_OF_LEDS ) && ( LED >= 0 ) )
    {
        /* Saturate each color channel. */
        G = ( G < 0 ) * 0 + ( G > 255 ) * 255 + ( ( G >= 0 ) && ( G <= 255 ) ) * G;
        R = ( R < 0 ) * 0 + ( R > 255 ) * 255 + ( ( R >= 0 ) && ( R <= 255 ) ) * R;
        B = ( B < 0 ) * 0 + ( B > 255 ) * 255 + ( ( B >= 0 ) && ( B <= 255 ) ) * B;

        ucLeds[ LED ][ GRN ] = G;
        ucLeds[ LED ][ RED ] = R;
        ucLeds[ LED ][ BLU ] = B;
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
        ucLedColor = ucLeds[ LED ][ color ];
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