#ifndef DEBUG_H
#define DEBUG_H

/* Peripheral includes. */
#include "stm32f4xx_tim.h"

/*-----------------------------------------------------------*/

typedef struct
{
    uint16_t usPatternClocks;
    uint16_t usUpdateLEDsClocks;
    uint16_t usCopyADCsFromDMAClocks;
} xDebugStats_t;

/*-----------------------------------------------------------*/

void vInitDebug( void );

#endif