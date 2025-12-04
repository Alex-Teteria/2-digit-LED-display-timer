# MIT License (MIT)
# Copyright (c) 2025 Oleksandr Teteria

from machine import Pin, Timer, PWM
import time
from ky040_encoder import RotaryEncoder
from seven_segment_led import SevenSegmentDisplay
from cd4094 import CD4094
from led_scale_util import *
import _thread


SEG_PINS = [0, 1, 2, 3, 4, 5, 6]  # виводи відповідно на сегменти a, b, c, d, e, f, g
DIGIT_PINS = [8, 7]  # to DIG.1 and DIG.2 for digit selection pins
TIME_SLEEP = 60  # time, in sec., after which sleep mode
TIME_OUT_DIVICE = 4 # time, in sec., час роботи виконавчого пристрою
pin_end_set = Pin(26, Pin.IN, Pin.PULL_UP)  # кнопка пуск-стоп-вихід_зі_сну
pin_multiplier = Pin(27, Pin.IN, Pin.PULL_UP)  # тумблер переключення режиму (сек.-хв.)
pin_out_device = Pin(13, Pin.OUT)  # на виконавчий пристрій
out_register = CD4094(data=11, clock=12, strobe=10, output_enable=9)
bits = 30 # number of LEDs in the scale
duty = 48_000 # яскравість led-шкали, max=65535

# Initialize a PWM object on GPIO pin 28
pwm_pin = PWM(Pin(28))
pwm_pin.freq(1000)  # Set frequency to 1000 Hz
pwm_pin.duty_u16(duty)

flag_run_timer = False

def rotary_event(val_max):
    '''callback for RotaryEncoder()
       val_max - (int), max value for RotaryEncoder.position
    '''
    # mode WRAP
    r.position %= val_max
    '''
    # mode BOUNDED
    if r.position > val_max-1:
        r.position = val_max-1
    elif r.position < 0:
        r.position = 0
    '''    
    print("Position:", r.position)


r = RotaryEncoder(clk_pin=14,
                  dt_pin=15,
                  debounce_ms=5,
                  callback= lambda pos, direction: rotary_event(100)
                  )


display = SevenSegmentDisplay(SEG_PINS, DIGIT_PINS)

tim_main = Timer()  # основний таймер
tim_led = Timer()   # таймер зміни значення екрана
tim_sleep = Timer() # таймер переходу в режим сну
tim_out_device = Timer() # таймер виходу на виконавчий пристрій
tim_scale = Timer()  # таймер led-шкали

class Value_timer:
    def __init__(self, value):
        self.value = value
        self.stop = False
    
    def dec_val(self):
        self.value -= 1
        if self.value < 0:
            self.value = 0
            
def set_time(start_value):
    r.position = start_value
    tim_sleep.init(period=1000 * TIME_SLEEP,
                  callback=tim_sleep_callback,
                  mode=Timer.ONE_SHOT
                  )
    while pin_end_set.value():
        value_led = value_to_num_bits(round(r.position * bits / 99), bits)
        out_register.shift_out(value_led, bits=bits)
        display.show(r.position, duration=0.1)
    tim_sleep.deinit()    

def timers_run(value):
    global flag_run_timer
    flag_run_timer = False
    mult = 1 if pin_multiplier.value() else 60
    timer.stop = False
    tim_main.init(period=value * 1000 * mult,
                  callback=tim_main_callback,
                  mode=Timer.ONE_SHOT
                  )
    tim_led.init(period=1000 * mult,
                 callback=tim_led_callback,
                 mode=Timer.PERIODIC
                 )
    while not timer.stop and pin_end_set.value():
        display.show(timer.value, duration=1.0)
    display.show(timer.value, duration=2)
    tim_main.deinit()
    tim_led.deinit()
    tim_scale.deinit()

def tim_main_callback(t):
    bell_run()
    tim_led.deinit()
    timer.stop = True

def bell_run():
    pin_out_device.value(1)
    tim_out_device.init(period=TIME_OUT_DIVICE * 1000,
                        callback=tim_out_device_callback,
                        mode=Timer.ONE_SHOT
                        )

def tim_out_device_callback(t):
    pin_out_device.value(0)

def tim_led_callback(t):
    timer.dec_val()

def debounce_pin(pin, num_delay):
    ''' вертає True, коли pin в стані "1"
        num_delay - множник для визначення затримки, затримка = num_delay * 10mc  
    '''
    if pin.value(): 
        for i in range(num_delay):
            time.sleep_ms(10)
            if not pin.value():
                break
        else:
            return True
    return False
                
def end_timer():
    tim_sleep.init(period=1000 * TIME_SLEEP,
                  callback=tim_sleep_callback,
                  mode=Timer.ONE_SHOT
                  )
    while pin_end_set.value():
        display.show('--', duration=0.2)
        display.show('  ', duration=0.2)
    tim_sleep.deinit()

def tim_sleep_callback(t):
    r.position = 0
    display.show('  ', duration=0.2)
    clear_scale(out_register, bits)
    while pin_end_set.value():
        machine.idle()
    print('end sleep')

# -------------------------------------
# func for second core
def run():
    mult = 1 if pin_multiplier.value() else 60
    while True:
        if flag_run_timer:
            value = value_to_num_bits(bits, bits)
            out_register.shift_out(value, bits=bits)
            value_scale.value = bits
            tim_scale.init(period=round(1000 * mult * timer.value / bits),
                 callback=tim_scale_callback,
                 mode=Timer.PERIODIC
                 )
                 
def tim_scale_callback(t):
    value_scale.dec_val()
    value = value_to_num_bits(value_scale.value, bits)
    out_register.shift_out(value, bits=bits)

# second core
_thread.start_new_thread(run, ())
# ---------------------------------------

value_scale = Value_timer(bits)
timer = Value_timer(0)
clear_scale(out_register, bits)
init_led_scale(out_register, bits)
while True:
    end_timer()
    while not debounce_pin(pin_end_set, 20):
        display.show('--', duration=0.1)
    set_time(timer.value)
    timer.value = r.position
    print(timer.value)
    while not debounce_pin(pin_end_set, 20):
        display.show(timer.value, duration=0.1)
        display.show('  ', duration=0.1)
    if timer.value:
        flag_run_timer = True
        timers_run(timer.value)
      

