# Individually Addressable LEDs

Example code using FreeRTOS to control individually addressable LEDs (SK6812).  This project also experiments with the DSP onboard the STM32F4-Discovery via audio syncing.

https://youtu.be/5Bn3eNN9Ki0

## Architecture

The idea is to use 2 different tasks to manage the LEDS. The tasks are listed below by priority (higher numbers will preempt lower numbers).

0.  Idle Task
    -  Does nothing
1.  Pattern Task
    - Decides what pattern to show
    - Calculates RGB color based on selected pattern
    - 10ms (100 Hz)
2.  Refresh Task
    - Refreshes the actual colors shown on the LED strip
        - Initiates PWM pulse train
    - 10ms (100 Hz)

## Peripherals

#### Timers

```
Timer 2: 44.1 kHz timer for sampling audio voltage data 
Timer 4: LED Data Line
```

#### GPIO:

```
C4: Audio voltage data in
B6: LED Data out
D12: LED4 (GRN) - Indicates stack overflow for vCreatePattern
D13: LED3 (ORG) - Indicates stack overflow for vUpdateLedStrip
D14: LED5 (RED)
D15: LED6 (BLU)
```

#### DMA:
```
DMA1 Stream 0: Transfer duty cycles to PWM module.
DMA2 Stream 0: Transfer ADC1->DR to buffer.
```

#### ADC's:
```
ADC1 Channel 14: Samples audio voltage data
```


## Resources

https://www.st.com/content/ccc/resource/technical/layouts_and_diagrams/schematic_pack/group1/60/36/18/8c/71/30/45/dc/MB997-F407VGT6-E01_Schematic/files/MB997-F407VGT6-E01_Schematic.pdf/jcr:content/translations/en.MB997-F407VGT6-E01_Schematic.pdf

https://www.st.com/resource/en/user_manual/dm00039084-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf

http://socialledge.com/sjsu/index.php/FreeRTOS_Tutorial

https://www.freertos.org/a00125.html

https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf

https://www.iar.com/support/resources/articles/using-iar-embedded-workbench-for-arm-and-the-cmsis-dsp-library/

https://stm32f4-discovery.net/2014/10/stm32f4-fft-example/

https://www.keil.com/pack/doc/CMSIS/DSP/html/group__RealFFT.html#ga5d2ec62f3e35575eba467d09ddcd98b5

https://community.st.com/s/question/0D50X00009XkibiSAB/stm32f207-adctimerdma-poor-peripheral-library-examples

## Notes
Use `xTaskCreateStatic` to create a task with the stack in a static location

Use `xTaskCreate` to create a task with the stack in a dynamic location

Use `vTaskDelayUntil` to schedule task to be run again

Use `uxTaskGetStackHighWaterMark` to see the highest of stack usage

Use `taskENTER_CRITICAL` to disable interrupts and tasks

Use `taskEXIT_CRITICAL` to re-enable interrupts and tasks

Use `portTICK_PERIOD_MS` to determine period of one clock tick/cycle