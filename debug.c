#include "debug.h"

/* Initialize globals */
xDebugStats_t xDebugStats = { 0 };

/**
  * @brief  Initialization debug statistics.
  * @param  None
  * @retval None
  */
void vInitDebug( void )
{
    /* Initialization structures. */
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* Initialize timer for debug purposes. */
    /* Enable the clock used for TIM12 (42MHz) */
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM12, ENABLE );

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535U;
    TIM_TimeBaseStructure.TIM_Prescaler = 63U;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0U;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit( TIM12, &TIM_TimeBaseStructure );

    /* TIM12 enable counter */
    TIM_Cmd(TIM12, ENABLE);
}
/*-----------------------------------------------------------*/