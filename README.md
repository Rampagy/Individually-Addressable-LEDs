# FreeRTOS Simple Example

Example code using FreeRTOS to control individually addressable LEDs (SK6812).

## Architecture

The idea is to use 3 different tasks to manage the LEDS.  The tasks are listed below by priority (higher numbers will preempt lower numbers).

0.  Idle Task
    -  Does nothing
1.  Pattern Task 
    - State chart to decide what pattern to show
    - 500ms (2 Hz)
2.  Calculation Task
    - Calculates the colors to be shown for each pattern
    - 50ms (20 Hz)
3.  Refresh Task
    - Refreshes the actual colors shown on the LED strip
        - Initiates PWM pulse train via DMA for the duty cycle of each period
    - 10ms (100 Hz)

## Peripherals

#### Timers

```
Timer 4: LED Data Line
```

#### GPIO:

```
B6: LED Data
D12: LED4 (GRN)
D13: LED3 (ORG) - Indicates stack overflow for vUpdatedLedStrip
D14: LED5 (RED)
D15: LED6 (BLU)
```

## Resources

http://socialledge.com/sjsu/index.php/FreeRTOS_Tutorial

https://www.freertos.org/a00125.html

https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf

## Notes
Use `xTaskCreateStatic` to create a task with the stack in a static location

Use `xTaskCreate` to create a task with the stack in a dynamic location

Use `vTaskDelayUntil` to schedule task to be run again

Use `uxTaskGetStackHighWaterMark` to see the highest of stack usage

Use `taskENTER_CRITICAL` to disable interrupts

Use `taskEXIT_CRITICAL` to re-enable interrupts

Use `portTICK_PERIOD_MS` to determine period of one clock tick/cycle