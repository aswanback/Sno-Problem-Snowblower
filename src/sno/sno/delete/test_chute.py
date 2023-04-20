
from stepper import Stepper, MotorInterfaceType

s = Stepper(MotorInterfaceType.DRIVER, 21, 19)
s.setAcceleration(50)
s.setMaxSpeed(300)
s.setSpeed(100)
while True:
    print('.',end='')
    s.runSpeed()
