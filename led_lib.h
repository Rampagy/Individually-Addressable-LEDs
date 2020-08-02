#ifndef LED_Lib_H
#define LED_Lib_H

#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include <stdbool.h>
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"


#define C_RED 0
#define C_GRN 1
#define C_BLU 2
extern const int STRIP_START;
extern const int STRIP_LENGTH;

//cos255: 0-255. Amplitude: 127 Vertical Offset: 127. 180 = 0 due to Voffset
extern uint8_t cos255[360];
uint8_t getCos(int val);

void setLED(int LED, uint8_t R, uint8_t G, uint8_t B);
uint8_t getLED(int LED, uint8_t color);

void fillStrip( uint8_t R, uint8_t G, uint8_t B);

void generateRamp(uint32_t start, uint32_t len, bool ramp, uint8_t R, uint8_t G,uint8_t B);
void crossfade(int start, int len, bool ramp, uint8_t R, uint8_t G,uint8_t B);
  void colorMixLED(int LED,  uint8_t R, uint8_t G,uint8_t B, uint8_t percent);
    
void updateStrand();

void LEDS_Init(void);
void WS2812_send(volatile uint8_t (*color)[3], uint16_t len);


#endif