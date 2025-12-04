"""
MicroPython driver for CD4094 shift register

This module allows easy interfacing with the CD4094 shift register.
Features:
  - Data shifting (MSB first)
  - Latching data into output register
  - Output enable/disable control

Compatible with ESP32, ESP8266, Raspberry Pi Pico, STM32, and other MicroPython boards.

Copyright (c) 2025 Oleksandr Teteria
License: MIT
"""

from machine import Pin
import time

class CD4094:
    def __init__(self, data, clock, strobe, output_enable):
        self.data = Pin(data, Pin.OUT)
        self.clock = Pin(clock, Pin.OUT)
        self.strobe = Pin(strobe, Pin.OUT)
        self.oe = Pin(output_enable, Pin.OUT)

        self.data.value(0)
        self.clock.value(0)
        self.strobe.value(0)
        self.oe.value(1)  # default: outputs enabled

    def shift_out(self, value, bits=8):
        '''Shift out bits to CD4094, MSB first'''
        for i in range(bits):
            bit = (value >> (bits - 1 - i)) & 1
            self.data.value(bit)
            self.clock.value(1)
            time.sleep_us(1)
            self.clock.value(0)
            time.sleep_us(1)
        # Latch the data
        self.strobe.value(1)
        time.sleep_us(1)
        self.strobe.value(0)
        time.sleep_us(1)

    def clear(self):
        '''Clear the data (all outputs low)'''
        self.shift_out(0)

    def enable_output(self):
        '''Enable outputs (active)'''
        self.oe.value(1)

    def disable_output(self):
        '''Disable outputs (Z-state)'''
        self.oe.value(0)

if __name__ == '__main__':
    # Example usage:
    cd = CD4094(data=11, clock=12, strobe=10, output_enable=13)  # Replace with your pins
    cd.enable_output()
    cd.shift_out(0b10101010)
    while True:
        cd.shift_out(0b10101010)
        time.sleep(0.5)
        cd.shift_out(0b00101010)
        time.sleep(0.5)
        
