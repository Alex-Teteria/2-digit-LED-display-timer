# Copyright (c) 2025 Oleksandr Teteria
# License: MIT

from cd4094 import CD4094
import time


def value_to_num_bits(value, bits):
    '''Перетворює число value в кількість біт "1", решта доповнює "0" до кількості bits
       приклад:
       value = 4, bits = 8
       вертає: 0b11110000
    '''
    a = 1
    for i in range(value - 1):
        a = a << 1 | a
    return a << (bits - value)

def rol(n, d, N):
    '''A left cyclic shift on a number n by d positions,
       considering a bit width of N
    '''
    return ((n << d) % (1 << N)) | (n >> (N - d))

def ror(n, d, N):
    '''A right cyclic shift on a number n by d positions,
       considering a bit width of N
    '''
    return ((n >> d) | ((n << (N - d)) % (1 << N)))

def init_led_scale(register, bits):
    '''register - примірник класу CD4094
       bits - кількість світлодіодів у шкалі (розмір регістру)
    '''
    value = 2**(bits-1)
    for i in range(bits-1):
        register.shift_out(value, bits=bits)
        value = ror(value, 1, bits)
        time.sleep(0.05)
    value = 1    
    for i in range(bits-1):
        register.shift_out(value, bits=bits)
        value = rol(value, 1, bits)
        time.sleep(0.05)
       
    for i in range(bits+1):
        value = value_to_num_bits(i, bits)
        register.shift_out(value, bits=bits)
        time.sleep(0.05)
        
    for i in range(bits, 0, -1):
        value = value_to_num_bits(i, bits)
        #print(bin(value))
        register.shift_out(value, bits=bits)
        time.sleep(0.05)
    
def clear_scale(register, bits):
    value = value_to_num_bits(0, bits)
    register.shift_out(value, bits=bits)
    

if __name__ == '__main__':
    out_register = CD4094(data=11, clock=12, strobe=10, output_enable=9)
    bits = 30

    clear_scale(out_register, bits)
    while True:
        init_led_scale(out_register, bits)

