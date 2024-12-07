# DRV2605(L) on STM32
![Waveform tested with this hardware setup](waveform.jpg)
Waveform tested with this hardware setup 

## Harware Setup
- NUCLEO-STM32H755ZI Development Board
- Adafruit DRV2605L Breakout Board ([DRV2506L Datasheet](https://www.ti.com/lit/gpn/drv2605l))
- Rotary Potentiometer
- Vybronics VG0840001D LRA
- 2-Position Slide Switch

## Operation Modes
This example is a demo of using STM32 microcontroller to control DRV2605L haptic driver via I2C and PWM. A 2-position slide swith connected to one of the GPIO14 determines whether the device is controlled via I2C to play internal vibration effect or via PWM to modulate the amplitude of vibration using a rotary potentiometer.

### I2C and Internal Waveform Library Effects
When the STM32 is connected to a external host via UART and the slide switch on GPIO14 is in the I2C position, the program reads a string of 1 to 115 from the UART and play the corresponding vibration from the internal Waveform Library Effects List (datasheet sec. `12.1.2`) in DRV2605. The I2C interface can also be used to control the driver with Real-Time Playback (RTP) Interface, which modulates the amplitude of vibration stored in a 8 bit I2C-writable register and triggered by wirting the `GO` bit in register `0x0C` via I2C. Refer to datasheet sec. `8.3.5.3` for more information on the RTP mode and sec. `8.3.5.6` for I2C triggering. 

### PWM Playback
When the slide switch on GPIO14 is in PWM position, the program reads the value from a rotary potentiometer connected to the ADC as the amplitude of the vibration motor. The PWM signal is then feed to the IN/TRIG pin on DRV2506. In this mode, the motor is always on. Refer to datasheet sec. `8.3.5.1` for more information.

## Programming [`main.c`](/CM7/Core/Src/main.c)
Every function is programmed in [/CM7/Core/Src/main.c](/CM7/Core/Src/main.c). This section goes through the source code to explain how to control the DRV2605 driver.

### Calibration
In the main function, the programm first perform the DRV2605 startup calibration procedure in the `/* USER CODE BEGIN 2 */` block. The startup calibration encures the motor is driven most efficiently. **Hold down the motor on the table on device startup to ensure the calibration result converges.** If the motor is moving due to the calibration vibration, the calibration will most likely fail. Optionally, read the `DIAG_RESULT` bit in register `0x00` to determine if the calibration succeeded.

The calibration procedure requires setting some register values related to the specification of the motor used. Please refer to datasheet sec. `8.5.6` for detailed description and calculation needed for the calibration parameters.

### I2C
On startup, the device defaults to and enter I2C mode. At the end of the `/* USER CODE BEGIN 2 */` block, two registers are written to select the internal waveform library (`LIBRARY_SEL[2:0]` in register `0x03`) for LRA motor and enter internal trigger mode (`MODE[2:0]` in register `0x01`). If using RTP mode to specify vibration amplitude via I2C instead of the internal library, write `MODE[2:0]` in register `0x01` to enter RTP mode and specify the amplitude by writing `RTP_INPUT[7:0] ` in register `0x02`. Refer to datasheet sec. `8.5.8` for more information and step-by-step guide (sec. `8.5.8.2.1`) for using the driver in RTP mode via I2C to modulate amplitude.

In I2C mode, whenever the microcontroller receives a string from UART, it writes the number received from UART to the first waveform sequencer register `0x04`. The number specify one vibration effect in the Waveform Library Effects List (datasheet sec. `12.1.2`). After the number is loaded into the wareform sequencer, the program writes the `GO` bit in register `0x0C` to trigger the vibration. A maximum of 8 waveforms can be laoded into the waveform sequencer and played at once by writing the `GO` bit.

The callback function `void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)` is called each time UART receives a new string. The function perform the aforementioned procedures.

### PWM
In PWM mode, the `MODE` bit is first written to the PWM/Analog mode and the `N_PWM_ANALOG` bit in `0x1D` is written to PWM mode. The driver can then be driven using PWM signal in the IN/TRIG pin.

## References
- [DRV2506L Datasheet](https://www.ti.com/lit/gpn/drv2605l)
- Waveform tested with this hardware setup ![Waveform tested with this hardware setup](waveform.jpg)
