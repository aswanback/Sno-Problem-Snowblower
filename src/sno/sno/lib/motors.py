from sno.lib.arduino import pinMode, analogWrite, IOType

class DriveMotor:
    def __init__(self, lpwm_pin, rpwm_pin):
        self.lpwm = lpwm_pin
        self.rpwm = rpwm_pin
        pinMode(self.lpwm, IOType.OUTPUT)
        pinMode(self.rpwm, IOType.OUTPUT)

    def forward(self, speed):
        ''' @speed [0,1] '''
        if speed > 1 or speed < 0:
            raise Exception('invalid motor speed')
        analogWrite(self.lpwm, speed)
        analogWrite(self.rpwm, 0)

    def backward(self, speed):
        ''' @speed [0,1] '''
        if speed > 1 or speed < 0:
            raise Exception('invalid motor speed')
        analogWrite(self.lpwm, 0)
        analogWrite(self.rpwm, speed)
    
    def move(self, speed):
        ''' @speed [-1,1] '''
        if speed > 1 or speed < -1:
            raise Exception('invalid motor speed')
        if speed > 0:
            self.forward(speed)
        elif speed < 0:
            self.backward(-speed)
        else:
            self.stop()
    
    def stop(self):
        analogWrite(self.lpwm, 0)
        analogWrite(self.rpwm, 0)

class AugerMotor:
    ''' Auger motor takes one pin for forward, others hardware defined '''
    def __init__(self, pin):
        self.pin = pin
        pinMode(self.pin, IOType.OUTPUT)

    def move(self, speed=1):
        ''' @speed [0,1] '''
        if speed > 1 or speed < 0:
            raise Exception('invalid motor speed')
        analogWrite(self.pin, speed)

    def stop(self):
        analogWrite(self.pin, 0)

if __name__ == '__main__':
    pass
    # from config import *
    # from arduino import pinMode, analogWrite, IOType
    # import time
    
    # motor = DriveMotor(LEFT_PWM1, LEFT_PWM2)
    # motor = DriveMotor(RIGHT_PWM1, RIGHT_PWM2)
    # motor = AugerMotor(AUGER_PWM)
    # motor.move(0.5)
    # time.sleep(5)
    # motor.stop()