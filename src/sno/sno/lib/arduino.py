import RPi.GPIO as GPIO
import time

# Set up the GPIO pins
GPIO.setmode(GPIO.BOARD)

gpio_pins = {
    8: 10,
    10: 19,
    16: 36,
    18: 12,
    22: 15,
    24: 18,
    26: 11,
    36: 16,
    38: 20,
    40: 21,
    19: 35,
    21: 29,
    23: 33,
    29: 31,
    31: 26,
    37: 13
}

pwm_pins = {
    11: 17,
    13: 27,
    32: 12,
    33: 13,
    35: 19
}

for pin in pwm_pins.values():
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, 1000)
    pwm.start(0)

def getPin(pin:int):
    if pin in pwm_pins:
        return pin
    elif pin in gpio_pins:
        return gpio_pins[pin]
    raise Exception('not a pin')

def pinMode(pin:int, iotype:int):
    if iotype == IOType.OUTPUT.value:
        GPIO.setup(pin, GPIO.OUT)
    elif iotype == IOType.INPUT.value:
        GPIO.setup(pin, GPIO.IN)
    else:
        raise ValueError('Invalid IOType')

def digitalWrite(pin:int, iovalue:int):
    if iovalue == IOValue.HIGH.value:
        GPIO.output(pin, GPIO.HIGH)
    elif iovalue == IOValue.LOW.value:
        GPIO.output(pin, GPIO.LOW)
    else:
        raise ValueError('Invalid IOValue')

def digitalRead(pin:int) -> bool:
    return GPIO.input(pin)

def analogWrite(pin:int, value:int):
    if pin not in pwm_pins:
        raise Exception('analogWrite: no pwm on pin')
    if value > 1 or value < 0:
        raise Exception(f'analogWrite: value should be [0,1], is {value}')

    duty_cycle = value * 100
    GPIO.output(pwm_pins[pin], GPIO.HIGH)
    pwm = GPIO.PWM(pwm_pins[pin], 1000)
    pwm.start(duty_cycle)

def micros():
    return time.time_ns() / 1e3 # time us
    
def delayMicroseconds(microseconds):
    start = time.time_ns()
    while True:
        elapsed = (time.time_ns() - start) / 1e3
        if elapsed >= microseconds:
            break

def delay(ms):
    time.sleep(ms/1000)