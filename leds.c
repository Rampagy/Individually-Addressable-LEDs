#include "leds.h"

/* Buffer that holds the LED information.   *
 * Format = [[GRB], [GRB], ...]             */
volatile uint8_t ucLeds[NUMBER_OF_LEDS][COLOR_CHANNELS] = {{ 0 }};

/* Buffer that hold the duty cycle information. */
static uint16_t usLedDutyCycleBuffer[TOTAL_PERIODS] = { 0 };

/* Task handle. */
TaskHandle_t xUpdateLedsHandle = NULL;


/**
  * @brief  Task that initiates the PWM stream to the individually addressable LEDs.
  * @note   Executes every 10ms resulting in a 100Hz update rate.
  * @retval None
  */
void vUpdateLedStrip( void * pvParameters  )
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10;
    UBaseType_t xAvailableStack = 0;

    /* Initialize the xLastWakeTime variable with the current time. */
     xLastWakeTime = xTaskGetTickCount();

     /* Create variable to hold the buffer index. */
     uint16_t usBufferIndex = 0;

    while ( 1 )
    {
        usBufferIndex = 0;

        /* Fill transmit buffer with correct compare values to achieve the
        correct pulse widths according to color values */
        for (uint16_t usI = 0; usI < NUMBER_OF_LEDS; usI++)
        {
            for (uint8_t ucJ = 0; ucJ < COLOR_CHANNELS; ucJ++)
            {
                for (uint8_t ucK = 0; ucK < 8; ucK++)
                {
                    uint8_t ucBit = ( ucLeds[usI][ucJ] & (0x80 >> ucK) ) >> (7 - ucK);
                    usLedDutyCycleBuffer[usBufferIndex] = LOW_THRESH + ucBit * ( (uint16_t)HIGH_THRESH - LOW_THRESH );
                    usBufferIndex++;
                }
            }
        }

        /* Set TIM4 CNT so an interrupt occurs to transfer the first byte. */
        TIM4->CNT = CLOCK_THRESH-1;

        /* Reset the pin used for transferring data. */
        GPIOB->ODR &= ~(0x01<<6);

        /* Clear the DMA flags. */
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_HTIF0);
        DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_FEIF0);

        /* Set the number of bytes to be transferred */
        DMA1_Stream0->NDTR = TOTAL_PERIODS;

        /* Start the DMA peripheral. */
        DMA_Cmd(DMA1_Stream0, ENABLE);

        /* Start the timer used for generating the PWM. */
        TIM_Cmd(TIM4, ENABLE);

        /* Check the stack size. */
        xAvailableStack = uxTaskGetStackHighWaterMark( xUpdateLedsHandle );

        if (xAvailableStack <= 10)
        {
            /* Turn orangle LED on if stack overflow is imminent/detected. */
            STM_EVAL_LEDOn( LED3 );
        }

        /* Wait for the next cycle. */
         vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
/*-----------------------------------------------------------*/


/**
  * @brief  Interrupt to handle the end of the DMA stream to the LEDs.
  * @param  None
  * @retval None
  */
uint32_t ulGetRandVal( void )
{
    return RNG_GetRandomNumber();
}
/*-----------------------------------------------------------*/

/**
  * @brief  Interrupt to handle the end of the DMA stream to the LEDs.
  * @param  None
  * @retval None
  */
void DMA1_Stream0_IRQHandler()
{
    /* Disable interrupts and other tasks from running during this interrupt. */
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    if( DMA_GetFlagStatus( DMA1_Stream0, DMA_FLAG_TCIF0 ) )
    {
        /* Clear DMA interrupt flags */
        DMA_ClearFlag( DMA1_Stream0, DMA_FLAG_TCIF0 );
        DMA_ClearFlag( DMA1_Stream0, DMA_FLAG_HTIF0 );
        DMA_ClearFlag( DMA1_Stream0, DMA_FLAG_FEIF0 );

        /* Disable timer 4 */
        TIM_Cmd( TIM4, DISABLE );

        /* Disable DMA stream 0 */
        DMA_Cmd( DMA1_Stream0, DISABLE );

        /* Reset the B6 pin to off. */
        GPIOB->ODR &= ~( 0x01 << 6 );
    }

    /* Re-enable interrupts and other tasks. */
    taskEXIT_CRITICAL_FROM_ISR( uxSavedInterruptStatus );
}
/*-----------------------------------------------------------*/


/**
  * @brief  Initialize the PWM stream to the individually addressable LEDs.
  * @param  None
  * @retval None
  */
void vInitLeds(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the clock used for DMA. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    /* Enable the clock used for GPIOB. */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* GPIO Configuration. */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Connect TIM4 pins to AF */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

    /* Enable the clock used for TIM4 (84MHz) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = CLOCK_THRESH;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel 1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = (uint32_t) 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* DMA1 channel 0 configuration. */
    DMA_DeInit(DMA1_Stream0);

    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR1;           // physical address of register to be loaded
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&usLedDutyCycleBuffer[0]; // physical address of data to load into register
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                     // shift data from memory to peripheral
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // automatically increase buffer index
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16 bits
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // 16 bits
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               // stop DMA feed after buffer size is reached
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
    DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);

    TIM_DMAConfig(TIM4, TIM_DMABase_CCR1, TIM_DMABurstLength_1Byte);
    TIM_SelectCCDMA(TIM4, ENABLE);
    TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);

    /* Set DMA interrupt when buffer transfers are complete. */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    /* Initalize the random number generator. */
    /* Call RNG_GetRandomNumber(void) to get a random number. */
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);
    RNG_Cmd(ENABLE);

}
/*-----------------------------------------------------------*/