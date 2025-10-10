from cd4094 import CD4094
import time

out_register = CD4094(data=7, clock=6, strobe=8, output_enable=5)

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

bits = 8
while True:
    for i in range(9):
        value = value_to_num_bits(i, bits)
        print(bin(value))
        out_register.shift_out(value)
        time.sleep(1)
        

