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

## File Structure

- `main_timer.py` — Main program logic
- `lib/ky040_encoder.py` — Library containing the encoder class
- `lib/seven_segment_led.py` — Library containing the display class

## Hardware Assembly

Assemble the timer according to the schematic using the listed components:
- RP2040 Zero microcontroller
- KY-040 rotary encoder
- E20561-L two-digit seven-segment LED display (common cathode)

## License

See the `License` file in the root of this repository.
