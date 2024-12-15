# Motor Drive Using PWM on Raspberry Pi Pico

## Introduction
This project demonstrates how to control a motor using Pulse Width Modulation (PWM) on a Raspberry Pi Pico board.

## Requirements
- Raspberry Pi Pico
- Motor driver (e.g., L298N)
- DC motor
- Power supply
- Jumper wires
- Breadboard

## Circuit Diagram
![Circuit Diagram](path/to/circuit-diagram.png)

## Setup
1. Connect the Raspberry Pi Pico to the motor driver.
2. Connect the motor to the motor driver.
3. Connect the power supply to the motor driver.
4. Connect the Raspberry Pi Pico to your computer via USB.

## Code

### Device tree

Device tree of Pico board include below

```
dts/arm/armv6-m.dtsi
dts/common/skeleton.dtsi
dts/common/freq.h
boards/raspberrypi/rpi_pico/rpi_pico.dts
boards/raspberrypi/rpi_pico/rpi_pico-common.dtsi
dts/arm/rpi_pico/rp2040.dtsi
dts/arm/rpi_pico/rpi_pico_common.dtsi
boards/raspberrypi/rpi_pico/rpi_pico-pinctrl.dtsi

```


## Testing
1. Upload the code to the Raspberry Pi Pico.
2. Run the code and observe the motor speed changes.

## Conclusion
This project demonstrates the basics of controlling a motor using PWM on a Raspberry Pi Pico. Further improvements can include adding direction control and speed feedback.

## References
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.org/documentation/pico/getting-started/)
- [PWM on Raspberry Pi Pico](https://datasheets.raspberrypi.org/pico/Pico-R3-A4-Pinout.pdf)