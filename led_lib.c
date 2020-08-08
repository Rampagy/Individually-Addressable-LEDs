/*
 * LED Library
 *
 * This library includes functions and tasks associated with sending the PWM
 * signals to the LEDS
 *
 * 1 tab == 4 spaces!
 */

/* File includes */
#include "LED_Lib.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define CLOCK_THRESH 106
#define HIGH_THRESH CLOCK_THRESH/3*2
#define LOW_THRESH CLOCK_THRESH/3

const int STRIP_START = 0;
const int STRIP_LENGTH = 148/2;//148 = full string

uint8_t cos255[360]={255,255,255,255,255,254,254,254,253,253,253,252,252,251,251,250,249,249,248,247,247,246,245,244,243,242,241,240,239,238,237,236,234,233,232,231,229,228,227,225,224,222,221,219,218,216,214,213,211,209,208,206,204,202,201,199,197,195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,160,158,156,154,152,150,147,145,143,141,139,136,134,132,130,128,125,123,121,119,116,114,112,110,108,105,103,101,99,97,95,92,90,88,86,84,82,80,78,76,74,72,70,68,66,64,62,60,58,56,54,53,51,49,47,46,44,42,41,39,37,36,34,33,31,30,28,27,26,24,23,22,21,19,18,17,16,15,14,13,12,11,10,9,8,8,7,6,6,5,4,4,3,3,2,2,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,1,1,2,2,2,3,3,4,4,5,6,6,7,8,8,9,10,11,12,13,14,15,16,17,18,19,21,22,23,24,26,27,28,30,31,33,34,36,37,39,41,42,44,46,47,49,51,53,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,95,97,99,101,103,105,108,110,112,114,116,119,121,123,125,128,130,132,134,136,139,141,143,145,147,150,152,154,156,158,160,163,165,167,169,171,173,175,177,179,181,183,185,187,189,191,193,195,197,199,201,202,204,206,208,209,211,213,214,216,218,219,221,222,224,225,227,228,229,231,232,233,234,236,237,238,239,240,241,242,243,244,245,246,247,247,248,249,249,250,251,251,252,252,253,253,253,254,254,254,255,255,255,255,255,255};
uint8_t getCos(int val){
  if(val>=360){val=val-(val/360)*360;}
  if(val<0){val = val+(val/360)*360;}
  if(val==360) val =0;
  return cos255[val];
}

uint8_t volatile leds[148][3];
uint16_t LED_BYTE_Buffer[3600];

void setLED(int LED, uint8_t R, uint8_t G, uint8_t B){
  if(LED<STRIP_LENGTH && LED>=0){
    leds[LED][0]=R;
    leds[LED][1]=G;
    leds[LED][2]=B;

  }

}

uint8_t getLED(int LED, uint8_t color){
  if(LED<STRIP_LENGTH && LED>=0){
    if( color<= C_BLU){
      return leds[LED][color];
    }else{
      return 0;
    }
  }else{
    return 0;
  }

}

void fillStrip( uint8_t R, uint8_t G, uint8_t B){
  for(int i = STRIP_START; i < (STRIP_LENGTH+STRIP_START); i++){
    setLED(i, R,G,B);
  }
}

//Ramp: if true 0 to 255, else 255 to 0
void generateRamp(uint32_t start, uint32_t len, bool ramp, uint8_t R, uint8_t G,uint8_t B){
  uint32_t counter =0;
  if(ramp){
    for(int i = start + len; i>  start;i--){
      leds[i][0]=leds[i][0]* i/len+R*(len-i)/len;
        leds[i][1]=leds[i][1]* i/len+G*(len-i)/len;
          leds[i][2]=leds[i][2]* i/len+B*(len-i)/len;
      //setLED(i, R*getCos((180/(len)*counter))/255,G*getCos((180/(len)*counter))/255,B*getCos((180/(len)*counter))/255,mix);
      counter++;
    }
  }else{
    for(int i = start; i< start + len ;i++){
      leds[i][0]=leds[i][0]* i/len+R*(len-i)/len;
        leds[i][1]=leds[i][1]* i/len+G*(len-i)/len;
          leds[i][2]=leds[i][2]* i/len+B*(len-i)/len;
      counter++;
    }
  }
}
//Ramp: if true 0 to 255, else 255 to 0
void crossfade(int start, int len, bool ramp, uint8_t R, uint8_t G,uint8_t B){
  uint32_t counter =0;
  if(ramp){
    for(int i = start + len; i>  start;i--){
      setLED(i,getLED(i,C_RED)* (counter)/len+R*(len-counter)/len,getLED(i,C_GRN)* (counter)/len+G*(len-counter)/len,getLED(i,C_BLU)* (counter)/len+B*(len-counter)/len);
      counter++;
    }
  }else{
    for(int i = start; i< start + len ;i++){
       setLED(i,getLED(i,C_RED)* (counter)/len+R*(len-counter)/len,getLED(i,C_GRN)* (counter)/len+G*(len-counter)/len,getLED(i,C_BLU)* (counter)/len+B*(len-counter)/len);
      counter++;
    }
  }
}

//mix new color into previous LED color by percent of new.
void colorMixLED(int LED,  uint8_t R, uint8_t G,uint8_t B, uint8_t percent){
  if(percent>100){
    percent =100;
  }

  setLED(LED, R*percent/100+getLED(LED,C_RED)*(100-percent)/100, G*percent/100+getLED(LED,C_GRN)*(100-percent)/100, B*percent/100+getLED(LED,C_BLU)*(100-percent)/100);
}

void LEDS_Init(void)
{

uint16_t PrescalerValue;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
DMA_InitTypeDef DMA_InitStructure;


   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

 //GPIO_InitTypeDef GPIO_InitStructure;


  //GPIOD enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /*-------------------------- GPIO Configuration ----------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Connect TIM4 pins to AF */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);

  // TIM4 enable, 84MHz
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
  TIM_OCInitStructure.TIM_Pulse = (uint32_t) LOW_THRESH;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* DMA1 Channel6 Config */
	DMA_DeInit(DMA1_Stream0);

        DMA_InitStructure.DMA_Channel = DMA_Channel_2;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM4->CCR1;	// physical address of Timer 3 CCR1
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&LED_BYTE_Buffer[0];		// this is the buffer memory
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;						// data shifted from memory to peripheral
	DMA_InitStructure.DMA_BufferSize =0;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// stop DMA feed after buffer size is reached
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;



	DMA_Init(DMA1_Stream0, &DMA_InitStructure);


  DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
  DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);

  TIM_DMAConfig(TIM4,TIM_DMABase_CCR1 ,TIM_DMABurstLength_1Byte);
  TIM_SelectCCDMA(TIM4,ENABLE);
  TIM_DMACmd(TIM4, TIM_DMA_CC1, ENABLE);



        NVIC_InitTypeDef NVIC_InitStructure;

          /* Enable and set User Button and IDD_WakeUP EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);

  RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
   RNG_Cmd(ENABLE);

}

/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 *
 * This will result in the RGB triplet passed by argument 1 being sent to
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 */
void WS2812_send(volatile uint8_t (*color)[3], uint16_t len)
{


	uint8_t j;
	uint8_t led;
	uint16_t memaddr;
	uint16_t buffersize;

	buffersize = (len*24)+50;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 50;				// reset buffer memory index

        ////////////////////
        // CRITICAL VALUE //
        ////////////////////

        /*This initial Value is critical to the operation of the code. Due to an
        odd glitch with the first output of the Output Compare unit, the first
        value must be large enough to trip the glitch, but small enough where it
        won't be visible.

        What I believe is happening:
        On the first large inputted value to the capture unit, it starts counting
        up. Partway into the count, it realizes that it needs to actually start
        comparing. At that point, it turns on the output. If faced with a small
        value, it pulses the output, but doesn't actaully compare the values.
        */
  	//LED_BYTE_Buffer[0] = 0;//Originally 12 from L1 Processor
       for (uint16_t i=memaddr-24;i<memaddr; i++){
            LED_BYTE_Buffer[i] = LOW_THRESH;
        }
        ////////////////////////
        // END CRITICAL VALUE //
        ////////////////////////
        led = 1;					// reset led index

	// fill transmit buffer with correct compare values to achieve
	// correct pulse widths according to color values
	while (len)
	{
		for (j = 0; j < 8; j++)					// GREEN data
		{
			if ( (color[led][1]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = HIGH_THRESH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = LOW_THRESH;	// compare value for logical 0
			}
			memaddr++;


		}

		for (j = 0; j < 8; j++)					// RED data
		{
			          if ( (color[led][0]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = HIGH_THRESH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = LOW_THRESH;	// compare value for logical 0
			}
			memaddr++;
		}

		for (j = 0; j < 8; j++)					// BLUE data
		{
			      if ( (color[led][2]<<j) & 0x80 )	// data sent MSB first, j = 0 is MSB j = 7 is LSB
			{
				LED_BYTE_Buffer[memaddr] = HIGH_THRESH; 	// compare value for logical 1
			}
			else
			{
				LED_BYTE_Buffer[memaddr] = LOW_THRESH;	// compare value for logical 0
			}
			memaddr++;
		}

		led++;
		len--;
	}

	// add needed delay at end of byte cycle, pulsewidth = 0
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}

        TIM4->CNT =  CLOCK_THRESH;
          GPIOB->ODR &= !(0x01<<6);

      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);
      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_HTIF0);
      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_FEIF0);

        DMA1_Stream0->NDTR = buffersize;

	//DMA_SetCurrDataCounter(DMA1_Channel4, buffersize); 	// load number of bytes to be transferred
	TIM_Cmd(TIM4,ENABLE);
          DMA_Cmd(DMA1_Stream0,ENABLE);
}

void DMA1_Stream0_IRQHandler(){

  if(DMA_GetFlagStatus(DMA1_Stream0,DMA_FLAG_TCIF0)){
    GPIOB->ODR &= !(0x01<<6);
      TIM_Cmd(TIM4,DISABLE);					// disable Timer 2
      DMA_Cmd(DMA1_Stream0, DISABLE); 			// disable DMA stream 6
      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);
      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_HTIF0);
      DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_FEIF0);
  }


}

void TIM1_IRQHandler(void){
   if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    }

}


void updateStrand(){
     WS2812_send(&leds[0], STRIP_LENGTH);
}

