#include "audio.h"

/* Initialize variables for fast fourier transform. */
arm_rfft_fast_instance_f32 S;
float32_t ufAdcSampleBuffer[ADC_SAMPLES] = { 0.0F };
uint16_t __usAdcIntermediateSampleBuffer[ADC_SAMPLES] = { 0U };


/**
  * @brief  Interrupt to handle the end of the DMA stream from ADCs.
  * @param  None
  * @retval None
  */
void DMA2_Stream0_IRQHandler( void )
{
    /* Disable interrupts and other tasks from running during this interrupt. */
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    /* Initialize start time.*/
    static uint16_t usStartTime;
    usStartTime = TIM12->CNT;

    /* DMA Stream Half Transfer interrupt */
    if( DMA_GetITStatus( DMA2_Stream0, DMA_IT_HTIF0 ) )
    {
        /* Clear DMA Stream Half Transfer interrupt pending bit */
        DMA_ClearITPendingBit( DMA2_Stream0, DMA_IT_HTIF0 );

        // Add code here to process first half of buffer (ping)
    }

    /* DMA Stream Transfer Complete interrupt */
    if( DMA_GetITStatus( DMA2_Stream0, DMA_IT_TCIF0 ) )
    {
        /* Clear DMA Stream Transfer Complete interrupt pending bit */
        DMA_ClearITPendingBit( DMA2_Stream0, DMA_IT_TCIF0 );

        /* Copy data from the intermediate buffer to the final buffer. */
        for ( uint16_t i = 0; i < ADC_SAMPLES; i++ )
        {
            ufAdcSampleBuffer[i] = (float32_t)__usAdcIntermediateSampleBuffer[i];
        }
    }

    /* Update the high water mark for task length. */
    if ( (uint16_t)TIM12->CNT - usStartTime > xDebugStats.usCopyADCsFromDMAClocks )
    {
        xDebugStats.usCopyADCsFromDMAClocks = (uint16_t)TIM12->CNT - usStartTime;
    }

    /* Re-enable interrupts and other tasks. */
    taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}
/*-----------------------------------------------------------*/



/**
  * @brief  Perform FFT.
  * @param  Variable to store the resulting FFT into.
  * @retval None
  */
void vPerformFFT( float32_t* ufFourierFrequency )
{
    /** @NOTE: ufFourierFrquency[0] is the dc offset.  To get the true dc offset
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

    /* Create array for storing frequency information. */
    float32_t ufComplexFFT[CFFT_SIZE] = { 0.0F };

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
    //float32_t temp = ufComplexFFT[0];

    /* Output of FFT is always complex, so convert to magnitude. */
    arm_cmplx_mag_f32( ufComplexFFT, ufFourierFrequency, RFFT_SIZE );

    /* Complex magnitude is not applicable to the DC offset. */
    //ufFourierFrequency[0] = temp;

    /* Rescale ufFourierFrequency to its true amplitudes.
     * I have no idea where these scalings come from, but it works. */
    float32_t scale = 1.0F / RFFT_SIZE;
    arm_scale_f32( ufFourierFrequency, scale, ufFourierFrequency, RFFT_SIZE );
}
/*-----------------------------------------------------------*/



/**
  * @brief  Initialization of the audio peripherals.
  * @param  None
  * @retval None
  */
void vInitAudio( void )
{
    /* Define GPIO init structure. */
    GPIO_InitTypeDef        GPIO_InitStructure;

    /* Define ADC init structures. */
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    DMA_InitTypeDef         DMA_InitStructure;

    /* Define TIM init structures. */
    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    /* Initialize the fast fourier transform. */
    (void) arm_rfft_fast_init_f32( &S, CFFT_SIZE );

    /* Enable the GPIO and ADC clocks. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // 84 MHz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 168 MHz
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 168 MHz
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 42 MHz

    /* Initialize for ADC1 on PC4 using IN14. */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Initialize PD3 for debugging purposes. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Enable the DMA Stream IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configMAX_SYSCALL_INTERRUPT_PRIORITY + 1U;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Clear the structure for later uses. */
    NVIC_InitStructure = ( const NVIC_InitTypeDef ){ 0 };

    /* TIM2 sample timer setup (44.1 kHz). */
    TIM_InitStructure.TIM_Prescaler = 0;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = ( ( 42000000U / ( TIM_InitStructure.TIM_Prescaler + 1 ) ) / SAMPLING_FREQUENCY ) - 1;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_InitStructure);

    /* TIM2 TRGO selection */
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);

    /* DMA1 channel 0 configuration. */
    DMA_DeInit( DMA2_Stream0 );

    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;                             // physical address of register to load into memory
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&__usAdcIntermediateSampleBuffer[0];      // physical address of memeory to be loaded
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                                     // shift data from peripheral to memeory
    DMA_InitStructure.DMA_BufferSize = ADC_SAMPLES;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                                     // automatically increase buffer index
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;                 // 16 bits
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;                         // 16 bits
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                             // Always keep this thing running
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    /* Enable DMA stream transfer complete interrupt. */
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* Init ADCs in independent mode, div clock by two */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;    // Not sure if this is needed
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* init ADC1: 12bit, single-conversion */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;               // 1 channel
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;         // Conversions triggered
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* Enable DMA Disable selection. */
    ADC1->CR2 |= ADC_CR2_DDS;

    ADC_RegularChannelConfig( ADC1, ADC_Channel_14, 1, ADC_SampleTime_15Cycles );
    ADC_DMACmd( ADC1, ENABLE );
    ADC_Cmd( ADC1, ENABLE );

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv( ADC1 );
}
/*-----------------------------------------------------------*/
