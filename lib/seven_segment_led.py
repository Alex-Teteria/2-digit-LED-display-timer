from machine import Pin
import time

SEG_PINS = [0, 1, 2, 3, 4, 5, 6]
DIGIT_PINS = [8, 7]

SEG_PATTERNS = {
    '0': [1,1,1,1,1,1,0],
    '1': [0,1,1,0,0,0,0],
    '2': [1,1,0,1,1,0,1],
    '3': [1,1,1,1,0,0,1],
    '4': [0,1,1,0,0,1,1],
    '5': [1,0,1,1,0,1,1],
    '6': [1,0,1,1,1,1,1],
    '7': [1,1,1,0,0,0,0],
    '8': [1,1,1,1,1,1,1],
    '9': [1,1,1,1,0,1,1],
    '-': [0,0,0,0,0,0,1],
    ' ': [0,0,0,0,0,0,0]
}

class SevenSegmentDisplay:
    def __init__(self, seg_pins, digit_pins):
        self.segments = [Pin(pin, Pin.OUT) for pin in seg_pins]
        self.digits = [Pin(pin, Pin.OUT) for pin in digit_pins]

    def show(self, text, duration=1.0, refresh_rate=0.005):
        # Manual right-alignment for 2 characters
        text = str(text)
        if len(text) < 2:
            text = '0' * (2 - len(text)) + text  # Pad with zero, or use ' ' for space
        else:
            text = text[-2:]  # Ensure at most 2 chars

        end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))
        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            for i, digit in enumerate(self.digits):
                pattern = SEG_PATTERNS.get(text[i], SEG_PATTERNS[' '])
                for seg_pin, value in zip(self.segments, pattern):
                    seg_pin.value(value)
                digit.value(0)
                time.sleep(refresh_rate)
                digit.value(1)

if __name__ == '__main__':
    # Example usage
    display = SevenSegmentDisplay(SEG_PINS, DIGIT_PINS)
    display.show('--', duration=2)
    while True:
        for num in range(0, 100):
            display.show(num, duration=0.2)
            
