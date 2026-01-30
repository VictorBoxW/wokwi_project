# Wowki Temperature-Controlled Fan (LED Simulation)

## Description
Arduino project that simulates a fan using an LED based on temperature measured with a DHT22 sensor. The temperature is displayed on a 3-digit 7-segment display.

### Fan (LED) behavior
- Below Tmin → LED off (fan off)
- Above Tmax → LED fully on (fan full speed)
- Between Tmin and Tmax → LED brightness PWM-controlled (25–99% duty cycle, simulating fan speed)
- Adjustable settings: Tmin and Tmax via buttons `[<]` and `[>]` with flashing display feedback. Values save after 5 seconds of inactivity.

## Features
- Automatic LED-based fan speed simulation
- Real-time Tmin/Tmax adjustment
- PWM LED control
- 7-segment display
- Interrupt-driven, direct hardware control (no Arduino libraries)

## Hardware
- Arduino Uno (or compatible)
- DHT22 temperature sensor
- 3-digit 7-segment display
- LED simulating a PWM-controlled fan
- Two push buttons

## Usage
1. Connect the hardware.
2. Upload the `.ino` file in Arduino IDE or use the Wokwi link provided.
3. Adjust Tmin/Tmax with `[<]` and `[>]`.
4. LED brightness adjusts automatically to simulate fan speed.
