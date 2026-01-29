# MIT License (MIT)
# Copyright (c) 2026 Oleksandr Teteria

from machine import Pin, Timer, I2C, SoftI2C
from machine import WDT
import time
import machine
import micropython
from ky040_encoder import RotaryEncoder
from tm1650 import TM1650
from bme280_float import BME280
from utils import *
import _thread
import gc
import json
import os

_wdt = None

micropython.alloc_emergency_exception_buf(256)

TIME_SLEEP = 60  # time, in sec., after which run bme280
TIME_OUT_DIVICE = 4 # time, in sec., час роботи виконавчого пристрою
pin_end_set = Pin(26, Pin.IN, Pin.PULL_UP)  # кнопка пуск-стоп-вихід_зі_сну
pin_multiplier = Pin(27, Pin.IN, Pin.PULL_UP)  # тумблер переключення режиму (сек.-хв.)
pin_out_device = Pin(28, Pin.OUT)  # на виконавчий пристрій
bits = 30 # number of LEDs in the scale

# час тримання шкали (ms) при зміні позиції мітки тиску енкодером,
# а вже потім знову - показ температури.
SHOW_POS_HOLD_MS = 800

# --- sleep-mode control for encoder processing on Core1 ---
sleep_mode = False          # True лише під час виконання _sleep_worker()
sleep_pos = 0               # останнє значення позиції (0..99) для відображення
sleep_pos_seq = 0           # лічильник змін (щоб не губити події між ядрами)

# "720": сотні = 7, два молодші розряди задає sleep_pos (00..99)
PRESSURE_BASE = 720

# щоб температура не перетирала короткий показ позиції
_show_pos_until_ms = 0

# збереження callback енкодера (на час сну відключаємо)
_encoder_cb_saved = None

# останні дані виміряного тиску
_pressure_old = 0


def _idle():
    # невеликий yield для обробки scheduled callbacks + зниження завантаження CPU
    try:
        machine.idle()
    except:
        time.sleep_ms(1)


# --- schedule guards (щоб не засмічувати чергу schedule) ---
_led_pending = False
_scale_pending = False
_sleep_pending = False
_scale_stop_req = False


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
    # print("Position:", r.position)


r = RotaryEncoder(clk_pin=14,
                  dt_pin=15,
                  debounce_ms=5,
                  callback= lambda pos, direction: rotary_event(100)
                  )

def encoder_pause_polling():
    # Зупинити внутрішній polling таймер енкодера (щоб Core1 керував опитуванням)
    try:
        r._poll_timer.deinit()
    except:
        pass

def encoder_resume_polling():
    # Відновити polling як було в ky040_encoder.py
    try:
        r._poll_timer.init(mode=Timer.PERIODIC, period=r._debounce_ms, callback=r._poll_rotary)
    except:
        pass


bus = SoftI2C(
    scl=Pin(5, pull=Pin.PULL_UP),
    sda=Pin(4, pull=Pin.PULL_UP),
    freq=100_000
)

try:
    bme = BME280(i2c=bus)
    bme.read_compensated_data(result=[0, 0, 0])
    bme_flag = True
except:
    bme_flag = False


i2c_scale = I2C(0, sda=Pin(0), scl=Pin(1), freq=100_000)
i2c_display = I2C(1, sda=Pin(2), scl=Pin(3), freq=4_000)
scale = TM1650(i2c_scale)
disp = TM1650(i2c_display)
disp.set_brightness(8)  # set max brightness for display
scale.set_brightness(8) # set max brightness for LED scale

# -----------------------------
# I2C0 (scale) global lock
scale_lock = _thread.allocate_lock()

def scale_clear():
    with scale_lock:
        scale.clear()

def scale_show_num_on_scale(n, fill=True):
    with scale_lock:
        scale.show_num_on_scale(n, fill=fill)

def scale_set_segments(chunks):
    with scale_lock:
        scale.set_segments(chunks)
# -----------------------------


timer = Value_timer(0)  # значення основного таймера

value_scale = Value_timer(bits) # значення для LED-шкали
scale_clear()

tim_led = Timer()    # таймер зміни значення екрана
tim_sleep = Timer()  # таймер переходу в режим сну
tim_scale = Timer()  # таймер led-шкали


        
def init_data_file():
    global PRESSURE_BASE

    path = 'data.json'
    data = {}
    bad = False
    need_write = False

    # 1) файл існує і не порожній?
    try:
        st = os.stat(path)
        size = st[6]  # st_size у MicroPython
        if size == 0:
            bad = True
    except OSError:
        bad = True

    # 2) якщо не bad — пробуємо прочитати JSON
    if not bad:
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            if not isinstance(data, dict):
                bad = True
        except:
            bad = True

    # 3) якщо bad — стартуємо з нуля і точно будемо писати
    if bad:
        data = {}
        need_write = True

    # 4) pressure_base: підхопити з файла, або додати якщо нема/битий
    if 'pressure_base' in data:
        try:
            PRESSURE_BASE = int(data['pressure_base'])
        except:
            data['pressure_base'] = int(PRESSURE_BASE)
            need_write = True
    else:
        data['pressure_base'] = int(PRESSURE_BASE)
        need_write = True

    # 5) якщо сенсор доступний — гарантуємо тиск у файлі
    if bme_flag:
        try:
            _, pressure, _ = bme.read_compensated_data(result=[0, 0, 0])
        except:
            pressure = 0  # тільки щоб ключі існували

        if 'pressure_current' not in data:
            data['pressure_current'] = pressure
            need_write = True

        if 'pressure_old' not in data:
            data['pressure_old'] = data['pressure_current']
            need_write = True

    # 6) пишемо тільки якщо треба
    if need_write:
        with open(path, 'w') as f:
            json.dump(data, f)


def _save_pressure_base_to_file():
    """Гарантовано зберегти PRESSURE_BASE в data.json (не чіпаючи інші поля)."""
    try:
        with open('data.json', 'r') as f:
            data = json.load(f)
    except:
        data = {}

    data['pressure_base'] = int(PRESSURE_BASE)

    with open('data.json', 'w') as f:
        json.dump(data, f)


def force_save_pressure_base():
    """Примусово оновити pressure_base в data.json (викликати перед reset)."""
    try:
        with open('data.json', 'r') as f:
            data = json.load(f)
    except:
        data = {}
    data['pressure_base'] = int(PRESSURE_BASE)
    with open('data.json', 'w') as f:
        json.dump(data, f)



def set_time(start_value):
    r.position = start_value
    tim_sleep.init(period=1000 * TIME_SLEEP,
                  callback=tim_sleep_callback,
                  mode=Timer.ONE_SHOT
                  )
    while pin_end_set.value():
        value_scale_num = round(r.position * bits / 99)
        disp.show(f'{r.position:04d}')
        if not r.position:
            scale_clear()
        scale_show_num_on_scale(value_scale_num)
        _idle()
        if gc.mem_free() < 10000:
            gc.collect()
    tim_sleep.deinit()    

            
def timers_run(value):
    global _scale_stop_req
    
    mult = 1 if pin_multiplier.value() else 60
    timer.stop = False
    tim_led.init(period=1000 * mult,
                 callback=tim_led_callback,
                 mode=Timer.PERIODIC
                 )
    disp.show(f'{timer.value:04d}')
    while not timer.stop and pin_end_set.value():
        _idle()
    disp.show(f'{timer.value:04d}')
    if timer.stop:
        bell_run()
    tim_led.deinit()
    _scale_stop_req = True
    scale_clear()
    time.sleep(2)


def bell_run():
    pin_out_device.value(1)
    print('BELL!')
    time.sleep(TIME_OUT_DIVICE)
    pin_out_device.value(0)


def _tim_led_worker(_):
    global _led_pending
    _led_pending = False
    timer.dec_val()
    disp.show(f'{timer.value:04d}')
    if not timer.value:
        timer.stop = True

def tim_led_callback(t):
    global _led_pending
    if _led_pending:
        return
    _led_pending = True
    try:
        micropython.schedule(_tim_led_worker, 0)
    except RuntimeError:
        _led_pending = False
        

def end_timer():
    tim_sleep.init(period=1000 * TIME_SLEEP,
                  callback=tim_sleep_callback,
                  mode=Timer.ONE_SHOT
                  )
    while pin_end_set.value():
        disp.show('----')
        time.sleep(0.2)
        disp.show('    ')
        time.sleep(0.2)
    tim_sleep.deinit()


def _sleep_worker(_):
    global _sleep_pending
    global sleep_mode, sleep_pos, sleep_pos_seq
    global PRESSURE_BASE, _show_pos_until_ms, _encoder_cb_saved
    global _need_reset
    global _wdt
    global _scale_stop_req
    global _pressure_old
    
    _sleep_pending = False
    
    if _wdt is None:
        _wdt = WDT(timeout=8000)  # ініціалізація, слідкуємо зависання протягом 8 секунд
    
    try:
        with open('data.json', 'r') as f:
            d = json.load(f)
        _pressure_old = d.get('pressure_old', 0)
    except:
        _pressure_old = 0
        
    # --- enter sleep mode ---
    tim_led.deinit()
    _scale_stop_req = True
    disp.show('    ')
    scale_clear()

    # Вимикаємо callback енкодера, щоб не було print/логіки з rotary_event у "сні"
    _encoder_cb_saved = getattr(r, "callback", None)
    try:
        r.callback = None
    except:
        pass

    # Зупиняємо внутрішній polling-таймер енкодера
    encoder_pause_polling()

    # Ставимо позицію в "дві молодші цифри" поточного PRESSURE_BASE (720 -> 20)
    r.position = PRESSURE_BASE % 100

    # Увімкнути режим: Core1 починає опитувати енкодер
    sleep_mode = True

    # Локальна змінна для відслідковування подій від Core1
    last_seq = sleep_pos_seq

    # --- цикл сну ---
    num_chunk = 0
    num_disp = 0
    num_write_pressure = 0

    while pin_end_set.value():
        _wdt.feed()  # годуємо watch dog
        # Якщо Core1 зафіксував зміну позиції — покажемо її на дисплеї
        if sleep_pos_seq != last_seq:
            last_seq = sleep_pos_seq
            disp.show(f'  {sleep_pos:02d}')
            _show_pos_until_ms = time.ticks_add(time.ticks_ms(), SHOW_POS_HOLD_MS)

        num_chunk = (num_chunk + 1) % 2
        num_disp = (num_disp + 1) % 8
        num_write_pressure = (num_write_pressure + 1) % 126_500  # ~3 години

        if not num_write_pressure and bme_flag:
            with open('data.json', 'r') as f:
                data = json.load(f)
            temperature, pressure, humidity = bme.read_compensated_data(result=[0, 0, 0])
            _pressure_old = data['pressure_current']
            data['pressure_old'], data['pressure_current'] = data['pressure_current'], pressure
            data["pressure_base"] = int(PRESSURE_BASE)
            with open('data.json', 'w') as f:
                json.dump(data, f)

        time.sleep(0.01)

        if bme_flag:
            bme280_show(num_chunk, num_disp)
        else:
            machine.idle()

    # --- exit sleep mode ---
    sleep_mode = False
    time.sleep_ms(10)   # дати Core1 один цикл вийти з гілки sleep_mode
    _save_pressure_base_to_file()
    scale_clear()
    print('end sleep')
    machine.reset()


def tim_sleep_callback(t):
    global _sleep_pending
    if _sleep_pending:
        return
    _sleep_pending = True
    try:
        micropython.schedule(_sleep_worker, 0)
    except RuntimeError:
        _sleep_pending = False


def bme280_show(num_chunk, num_disp):
    global _show_pos_until_ms, PRESSURE_BASE

    temperature, pressure, humidity = bme.read_compensated_data(result=[0, 0, 0])

    # якщо зараз показуємо sleep_pos — не перетираємо дисплей температурою
    if time.ticks_diff(_show_pos_until_ms, time.ticks_ms()) <= 0:
        if not num_disp:
            disp.show('  ' + str(round(temperature)))

    pressure_old = round(_pressure_old / 133.3224) - PRESSURE_BASE
    pressure_new = round(pressure / 133.3224) - PRESSURE_BASE

    if pressure_old < pressure_new:
        pressure_old -= 1

    if pressure_new == pressure_old:
        scale_clear()
        scale_show_num_on_scale(pressure_new, fill=False)
    else:
        chunks_1 = two_numbers_into_led_segment(pressure_old, pressure_new)
        chunks_2 = two_numbers_into_led_segment(pressure_old, pressure_new-1)
        if num_chunk:
            scale_set_segments(chunks_1)
        else:
            scale_set_segments(chunks_2)


# -------------------------------------
# func for second core
def run():
    global sleep_mode, sleep_pos, sleep_pos_seq, PRESSURE_BASE
    global _scale_stop_req

    last_pos = None

    while True:
        if _scale_stop_req:
            _scale_stop_req = False
            try:
                tim_scale.deinit()
            except:
                pass
        # --- режим сну: Core1 опитує енкодер і формує PRESSURE_BASE ---
        if sleep_mode:
            # оновлюємо внутрішній стан енкодера (той самий алгоритм з ky040_encoder.py)
            try:
                r._poll_rotary(None)
            except:
                pass

            pos = r.position % 100
            r.position = pos  # фіксуємо 0..99

            if last_pos is None:
                last_pos = pos
            elif pos != last_pos:
                last_pos = pos
                PRESSURE_BASE = 700 + pos
                sleep_pos = pos
                sleep_pos_seq += 1  # подія для Core0

            # частота опитування ~debounce_ms
            time.sleep_ms(getattr(r, "_debounce_ms", 5))
            continue

        # при виході зі сну скидаємо last_pos, щоб перший рух знов зловити коректно
        last_pos = None

        # --- режим шкали таймера ---
        if timer.flag_run_timer:
            timer.flag_run_timer = False
            mult = 1 if pin_multiplier.value() else 60
            scale_show_num_on_scale(bits)
            value_scale.value = bits
            tim_scale.init(period=round(1000 * mult * timer.value / bits),
                 callback=tim_scale_callback,
                 mode=Timer.PERIODIC
                 )

        time.sleep_ms(5)


def _tim_scale_worker(_):
    global _scale_pending
    _scale_pending = False
    value_scale.dec_val()
    scale_show_num_on_scale(value_scale.value)

def tim_scale_callback(t):
    global _scale_pending
    if _scale_pending:
        return
    _scale_pending = True
    try:
        micropython.schedule(_tim_scale_worker, 0)
    except RuntimeError:
        _scale_pending = False

# second core
_thread.start_new_thread(run, ())
# ---------------------------------------

def main_run():
    end_timer()
    while not debounce_pin(pin_end_set, 20):
        disp.show('----')
    set_time(timer.value)
    timer.value = r.position
    print(timer.value)
    while not debounce_pin(pin_end_set, 20):
        disp.show(f'{timer.value:04d}')
        time.sleep(0.1)
        disp.show('    ')
        time.sleep(0.1)
    if timer.value:
        timer.flag_run_timer = True
        timers_run(timer.value)

init_data_file()

while True:
    main_run()

