import rclpy
import rclpy.node
from sno_interfaces.msg import FlutterControl, Mode, Motors, Stepper
from sno.lib.config import *
from sno.lib.arduino import digitalRead, getPin, pinMode, IOType

class HandleNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('handle_node')
        self.log = self.get_logger().info
        self.create_subscription(Mode, '/mode', self.mode_cb, 10)
        self.motor_pub = self.create_publisher(Motors, '/motors', 1)
        self.stepper_pub = self.create_publisher(Stepper, '/stepper', 1)
        self.create_timer(HANDLE_UPDATE_RATE, self.handle_cb)
        self.pin_list = [HANDLE_LEFT, HANDLE_RIGHT, HANDLE_DIRECTION, HANDLE_CHUTE_LEFT, HANDLE_CHUTE_RIGHT, HANDLE_AUGER_ON]
        self.pins = [getPin(p) for p in self.pin_list]
        for p in self.pins:
            pinMode(p, IOType.INPUT)
        self.auto_mode = False
        self.last_values = [None]*6
    
    def mode_cb(self, data):
        self.auto_mode = data.auto_mode
    
    def handle_cb(self):
        if not self.auto_mode:
            left, right, direction, chute_left, chute_right, auger_on = [digitalRead(p) for p in self.pins]
            direction = not direction # hardware hack
            auger_on = not auger_on # hardware hack
            self.log(f'left: {left}\n right:{right}\ndir:{direction}\nchute_left:{chute_left}\nchute_right{chute_right}\nauger_on:{auger_on}')

            d = 1 if direction else -1
            l = 1 if left else 0
            r = 1 if right else 0
            msg = Motors()
            msg.left = float(d*l)
            msg.right = float(d*r)
            msg.auger = auger_on
            msg.enable = [True]*3
            self.motor_pub.publish(msg)

            msg = Stepper()
            msg.left = chute_left
            msg.right = chute_right
            self.stepper_pub.publish(msg)
        


def main():
    rclpy.init()
    node = HandleNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()