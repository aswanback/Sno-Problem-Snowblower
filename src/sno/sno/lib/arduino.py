
from enum import Enum
import os
import subprocess
from adafruit_blinka.board.librecomputer.aml_s905x_cc_v1 import *
from digitalio import DigitalInOut, Direction
import time

class IOType(Enum):
    OUTPUT = 1
    INPUT = 0

class IOValue(Enum):
    HIGH = 1
    LOW = 0

def setupPWM():
    commands = [
        'echo pi | sudo -S echo sudo_login',
        'sudo ldto enable pwm-a',
        'sudo ldto enable pwm-ao-9',
        'sudo ldto enable pwm-ef',
        'echo 0 | sudo tee /sys/class/pwm/pwmchip0/export',
        'echo 0 | sudo tee /sys/class/pwm/pwmchip2/export',
        'echo 1 | sudo tee /sys/class/pwm/pwmchip2/export',
        'echo 0 | sudo tee /sys/class/pwm/pwmchip4/export',
        'echo 1 | sudo tee /sys/class/pwm/pwmchip4/export'
    ]
    for command in commands:
        subprocess.run(command, stderr=subprocess.PIPE, stdout=subprocess.DEVNULL, shell=True)
setupPWM() 

gpio_pins = {8: P8, 
             10: P10, 
             16: P16, 
             18: P18, 
             22: P22,
             24: P24,
             26: P26,
             36: P36,
             38: P38, 
             40: P40, 
             19: P19, 
             21: P21, 
             23: P23, 
             29: P29, 
             31: P31, 
             37: P37}

pwm_pins = {
    11: (4, 0), # 11086c0 pwm0 pwmchip4   11
    13: (4, 1), # 11086c0 pwm1 pwmchip4   13
    32: (2, 0), # 8100550 pwm0 pwmchip2   32
    33: (0, 0), # 1108550 pwm0 pwmchip0   33
    35: (2, 1), # 8100550 pwm1 pwmchip2   35
}

def getPin(pin:int):
    if pin in pwm_pins:
        return pin
    elif pin in gpio_pins:
        return DigitalInOut(gpio_pins[pin])
    raise Exception('not a pin')


def pinMode(pin:DigitalInOut, iotype:IOType):
    if isinstance(pin,DigitalInOut):
        pin.direction = Direction.OUTPUT if iotype.value == IOType.OUTPUT.value else Direction.INPUT
    else:
        print(f'not a digital pin {pin}')

def digitalWrite(pin:DigitalInOut, iovalue:int) -> None:
    if isinstance(pin, DigitalInOut):
        pin.value = 1 if iovalue == IOValue.HIGH.value else 0
    else:
        raise Exception('digitalWrite: not a digital pin')

def digitalRead(pin:DigitalInOut) -> bool:
    if not isinstance(pin, DigitalInOut):
        raise Exception('digitalRead: not a digital pin')
    if pin.direction != Direction.INPUT:
        raise Exception('digitalRead: pin in output mode')
    return pin.value

def analogWrite(pin:int, value:int):
    ''' @value [0, 1] '''
    if pin not in pwm_pins:
        raise Exception('analogWrite: no pwm on pin')
    if value > 1 or value < 0:
        raise Exception(f'analogWrite: value should be [0,1], is {value}')
    
    chip, channel = pwm_pins[pin]
    path = f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"
    period = 1000000 # int(1/frequency * 1000000000)
    duty_cycle = int(period * value)
    commands = [
        f'echo pi | sudo -S echo sudo_login',
        f'echo 0 | sudo tee {path}/enable',
        f'echo {period} | sudo tee {path}/period',
        f'echo {duty_cycle} | sudo -S tee {path}/duty_cycle',
        f'echo 1 | sudo -S tee {path}/enable'
    ]
    for command in commands:
        subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, shell=True)

def micros():
    return time.time_ns() / 1e3 # time us
    
def delayMicroseconds(microseconds):
    start = time.time_ns()
    while True:
        elapsed = (time.time_ns() - start) / 1e3
        if elapsed >= microseconds:
            break

def delay(ms):
    delayMicroseconds(ms*1000)