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
    (void) arm_rfft_fast_init_f32( &S, FFT_SIZE );

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
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
