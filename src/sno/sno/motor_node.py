import rclpy
import rclpy.node
from sno_interfaces.msg import Motors, Ultrasonics
from sno.lib.motors import DriveMotor, AugerMotor
from sno.lib.config import *

class MotorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('motor_node')
        self.log = self.get_logger().info
        self.create_subscription(Motors, '/motors', self.motor_cb, 10)
        self.create_subscription(Ultrasonics, '/ultrasonic_stop', self.ultrasonics_cb, 10)
        self.left_motor = DriveMotor(LEFT_PWM1, LEFT_PWM2)
        self.right_motor = DriveMotor(RIGHT_PWM1, RIGHT_PWM2)
        self.auger_motor = AugerMotor(AUGER_PWM)
        self.ultrasonic_stop = False

        self.last_values = [None, None, None]
    
    def ultrasonics_cb(self, data:Ultrasonics):
        self.ultrasonic_stop = data.stop

    def motor_cb(self, data:Motors):
        if self.ultrasonic_stop:
            self.left_motor.stop()
            self.right_motor.stop()
            self.auger_motor.stop()
            return
        if data.enable[0]:
            if data.left != self.last_values[0]:
                self.last_values[0] = data.left
                self.left_motor.move(data.left)
        if data.enable[1]:
            if data.right != self.last_values[1]:
                self.last_values[1] = data.right
                self.right_motor.move(data.right)
        if data.enable[2]:
            if data.auger != self.last_values[2]:
                self.last_values[2] = data.auger
                if data.auger:
                    self.auger_motor.move(DEFAULT_AUGER_SPEED_PERCENT)
                else:
                    self.auger_motor.stop()

def main():
    rclpy.init()
    node = MotorNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()