from machine import Pin, Timer
import utime
import micropython


micropython.alloc_emergency_exception_buf(256)

class RotaryEncoder:
    # State table for rotary encoder transitions (Gray code)
    _STATE_TABLE = [
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    ]

    def __init__(self, clk_pin, dt_pin, sw_pin=None, callback=None, debounce_ms=5, btn_debounce_ms=50):
        self.clk = Pin(clk_pin, Pin.IN, Pin.PULL_UP)
        self.dt = Pin(dt_pin, Pin.IN, Pin.PULL_UP)
        self.sw = Pin(sw_pin, Pin.IN, Pin.PULL_UP) if sw_pin is not None else None

        self.position = 0
        self.callback = callback
        self._debounce_ms = debounce_ms
        self._btn_debounce_ms = btn_debounce_ms

        self._last_state = (self.clk.value() << 1) | self.dt.value()
        self._last_btn_time = 0
        self._step_count = 0

        # schedule state
        self._sched_pending = False
        self._cb_dir = 0
        self._cb_button = False
        
        # Use a Timer for periodic polling (debounced)
        self._poll_timer = Timer()
        self._poll_timer.init(mode=Timer.PERIODIC, period=self._debounce_ms, callback=self._poll_rotary)

        if self.sw:
            self.sw.irq(trigger=Pin.IRQ_FALLING, handler=self._button_handler)

    def _poll_rotary(self, t):

        clk_val = self.clk.value()
        dt_val = self.dt.value()
        state = (clk_val << 1) | dt_val
        idx = (self._last_state << 2) | state
        direction = self._STATE_TABLE[idx]

        if direction != 0:
            self._step_count += direction
            # Only update position every 4 steps (1 detent)
            if abs(self._step_count) >= 4:
                detent_direction = int(self._step_count / 4)
                self.position += detent_direction
                self._step_count = 0
                self._cb_dir = detent_direction
                self._cb_button = False
                self._schedule_cb()

            self._last_state = state
        else:
            self._last_state = state

    def _button_handler(self, pin):
        current_time = utime.ticks_ms()
        if utime.ticks_diff(current_time, self._last_btn_time) < self._btn_debounce_ms:
            return
        self._last_btn_time = current_time
        if self.sw.value() == 0:  # Button pressed
            self._cb_dir = 0
            self._cb_button = True
            self._schedule_cb()

    def _schedule_cb(self):
        if not self.callback:
            return
        if self._sched_pending:
            return
        self._sched_pending = True
        try:
            micropython.schedule(self._run_cb, 0)
        except RuntimeError:
            self._sched_pending = False

    def _run_cb(self, _):
        self._sched_pending = False
        try:
            if self._cb_button:
                self.callback(self.position, 0, button=True)
            else:
                self.callback(self.position, self._cb_dir)
        except Exception:
            pass

    def get_position(self):
        return self.position
      
# Example usage:
# Connect KY-040 CLK to GP15, DT to GP14
def rotary_event(pos, direction, button=False):
    if button:
        print("Button pressed at position:", pos)
    else:
        if encoder.position > 19:
            encoder.position = 19
        elif encoder.position < 1:
            encoder.position = 1
        print("Position:", pos, "Direction:", "CW" if direction > 0 else "CCW")


if __name__ == '__main__':
    
    encoder = RotaryEncoder(clk_pin=15,
                            dt_pin=14,
                            sw_pin=26,
                            debounce_ms=5,
                            callback=rotary_event)
    encoder.position = 10
    while True:
        utime.sleep(0.1)
        
    # You can also poll encoder.get_position() if you don't want to use callback