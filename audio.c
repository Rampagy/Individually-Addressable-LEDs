#include "audio.h"

/* Initialize variables for fast fourier transform. */
arm_rfft_fast_instance_f32 S;
float32_t ufAdcSampleBuffer[ADC_SAMPLES] = { 0.0 };
uint16_t usAdcSampleIndex = 0;
uint8_t  ucAdcConversionComplete = 0;
uint8_t  ucAdcBufferFull = 0;

/**
  * @brief  Interrupt to handle the end of the ADC conversion.
  * @param  None
  * @retval None
  */
void ADC_IRQHandler( void )
{
    /* Disable interrupts and other tasks from running during this interrupt. */
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    if ( ADC_GetITStatus( ADC1, ADC_IT_EOC ) != RESET )
    {
        /* Clear ADC end of conversion interrupt flag */
        ADC_ClearITPendingBit( ADC1, ADC_IT_EOC );

        /* Set the conversion complete flag to 1. */
        ucAdcConversionComplete = 1;
    }

    /* Re-enable interrupts and other tasks. */
    taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}
/*-----------------------------------------------------------*/


/**
  * @brief  Interrupt to handle the 44.1 kHz sampling.
  * @param  None
  * @retval None
  */
void TIM8_BRK_TIM12_IRQHandler( void )
{
    /* Disable interrupts and other tasks from running during this interrupt. */
    //UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    if ( TIM_GetITStatus( TIM12, TIM_IT_Update ) != RESET )
    {
        /* Reset interrupt flag. */
        TIM_ClearITPendingBit( TIM12, TIM_IT_Update );

        /* Debugging Purposes. */
        GPIO_ToggleBits( GPIOD, 1 << 3 );

        /* If ADC conversion is complete save the value */
        if ( ucAdcConversionComplete )
        {
            /* Save the ADC sample. */
            ufAdcSampleBuffer[usAdcSampleIndex] = (float32_t)(ADC1->DR & 0xFFF) - (float32_t)(0xFFF / 2);
            ucAdcConversionComplete = 0;

            /* Start the next ADC conversion. */
            ADC_SoftwareStartConv( ADC1 );
        }
        else
        {
            /* Turn on the red LED to indicate the ADC sample timer was
             * completed before the ADC conversion.
             */
            STM_EVAL_LEDOn( LED5 );
        }

        /* Increment index counter. */
        usAdcSampleIndex++;

        /* Clear index counter on rollover. */
        if ( usAdcSampleIndex >= ADC_SAMPLES )
        {
            usAdcSampleIndex = 0;
            ucAdcBufferFull = 1;
            TIM_Cmd( TIM12, DISABLE );
        }
    }

    /* Re-enable interrupts and other tasks. */
    //taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
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

    /* Define TIM init structures. */
    NVIC_InitTypeDef        NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    /* Initialize the fast fourier transform. */
    (void) arm_rfft_fast_init_f32( &S, FFT_SIZE );

    /* Enable the GPIO and ADC clocks. */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Initialize for ADC1 on PC4 using IN14. */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
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

    /* Init ADCs in independent mode, div clock by two */
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* init ADC1: 12bit, single-conversion */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = 0;
    ADC_InitStructure.ADC_ExternalTrigConv = 0;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_480Cycles);

    /* Enable ADC end of conversion interrupts */
    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

    /* Configure ADC interrupt (higher numbers preempt lower numbers). */
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_Init(&NVIC_InitStructure);

    ADC_Cmd( ADC1, ENABLE );

    /* Enable the timer clock (42MHz). */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    /* 44.1 kHz sample timer setup. */
    TIM_InitStructure.TIM_Prescaler = 2U;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = ( ( 84000000U / (TIM_InitStructure.TIM_Prescaler+1) ) / SAMPLING_FREQUENCY ) - 1;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM12, &TIM_InitStructure);

    /* Reset interrupt flag. */
    TIM_ClearITPendingBit( TIM12, TIM_IT_Update );

    /* Timer 12 Interrupt Config */
    NVIC_InitStruct.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x05;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable interrupt. */
    TIM_ITConfig(TIM12, TIM_IT_Update, ENABLE);
}
/*-----------------------------------------------------------*/