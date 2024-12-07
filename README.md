# DRV2605(L) on STM32

## Harware Setup
- NUCLEO-STM32H755ZI Development Board
- Adafruit DRV2605L Breakout Board
- Rotary Potentiometer
- Vybronics VG0840001D LRA
- 2-Position Slide Switch

## Programming
This example is a demo of using STM32 microcontroller Dto control RV2605L haptic driver via I2C and PWM. A 2-position slide swith connected to one of the GPIO14 determines whether the device is controlled via I2C to play internal vibration effect or via PWM to modulate the strength of vibration using a rotary potentiometer.

### I2C and internal Waveform Library Effects
When the STM32 is connected to a external host via UART, the program reads a string of 1 to 115 from the UART and play the corresponding vibration from the internal Waveform Library Effects List (datasheet sec. `12.1.2`) in DRV2605. The I2C interface can also be used to control the driver with Real-Time Playback (RTP) Interface, which modulates the strength of vibration stored in a 8 bit I2C-writable register and triggered by wirting the GO bit in register 0x0C via I2C.