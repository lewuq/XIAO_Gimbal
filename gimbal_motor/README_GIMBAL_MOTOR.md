# gimbal_motor

Headless XIAO STM32C5 controller for the MS3506 yaw and MS3008 pitch motors.

- No round display, touch, SPI display, or I2C touch dependency.
- CAN motor loop: preemptive priority 0, 10 ms period.
- UART RX/TX loop: preemptive priority 1, 5 ms service period.
- Motor telemetry: 20 Hz (50 ms), interrupt-driven TX ring buffer.
- UART1: D6/PA9 TX, D7/PA10 RX, 115200 8N1.
- Command flags: bit 0 CENTER, bit 1 AUTO, bit 2 PERSON, bit 3 CAL.

Build with `platformio run`. The UF2 output is
`.pio/build/seeed-xiao-stm32c5/firmware.uf2`.
