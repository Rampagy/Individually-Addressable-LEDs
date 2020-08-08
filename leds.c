#include "leds.h"

/* Buffer that holds the LED information. */
uint8_t volatile ucLeds[148][3];

/* Buffer that hold the duty cycle information. */
uint16_t usLedDutyCycleBuffer[TOTAL_PERIODS] = {{ 0 }};

/* Function used to initialize the PWM stream to the individually addressable
LEDs. */
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
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // used to be pull up
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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

    /* DMA1 channel 2 configuration. */
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
    DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);

    TIM_DMAConfig(TIM4, TIM_DMABase_CCR1, TIM_DMABurstLength_1Byte);
    TIM_SelectCCDMA(TIM4, ENABLE);
    TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);

    /* Set DMA interrupt when buffer transfers are complete. */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    /* Set interrupt on every completion of pulse.  I believe it's used to
    initiate the DMA transfer for the next byte (LED duty cycle).  There is no
    actual interrupt code implemented.  I believe it's used purely for
    initiating the DMA transfer of the next byte) */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
    //RNG_Cmd(ENABLE); // RNG_GetRandomNumber(void)
}
/*-----------------------------------------------------------*/