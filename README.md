# Countdown Timer Project

## Description

This project is a hardware-software countdown timer with two selectable time ranges:  
- **1 to 99 seconds** (step: 1 second)  
- **1 to 99 minutes** (step: 1 minute)  

The timer is built around the following key components:
- **RP2040 Zero microcontroller**
- **KY-040 rotary encoder**
- **E20561-L seven-segment LED display** (2-digit, dynamic indication, common cathode)

## Features

1. **Dual Time Ranges:** Switchable via a toggle (seconds/minutes)
   - 1..99 seconds (step: 1 second)
   - 1..99 minutes (step: 1 minute)
2. **Time Setting:** Set the countdown period using the KY-040 rotary encoder
3. **Display:** Shows the set time and the timer status on a 2-digit seven-segment LED display
4. **Output:** Provides an output signal for an actuator (e.g., bell, light indicator, etc.)
5. **LED scale:** LED scale (30 LEDs) linearly highlights the remaining time

## File Structure

- `main_timer.py` — Main program logic
- `lib/ky040_encoder.py` — Library containing the encoder class
- `lib/seven_segment_led.py` — Library containing the display class
- `lib/cd4094.py` — containing the driver for interfacing with the CD4094 shift register

## Hardware Assembly

Assemble the timer according to the schematic using the listed components:
- RP2040 Zero microcontroller
- KY-040 rotary encoder
- E20561-L two-digit seven-segment LED display (common cathode)
- To build the LED scale, three ten-segment LED indicators of the OSX10201 type were used

## Principle of implementing dynamic LED indication using CD4094 type shift registers  
![LED_scale_2](https://github.com/user-attachments/assets/33b011c1-2a66-4bda-9090-ad85f0270192)

## License

See the `License` file in the root of this repository.
