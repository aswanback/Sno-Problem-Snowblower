

import rclpy
import rclpy.node
from sno_interfaces.msg import Mode
from sno.lib.config import *
from sno.lib.arduino import digitalRead, getPin, pinMode, IOType
from std_msgs.msg import Bool

class ModeNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('mode_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(Mode, '/mode', 1)
        self.create_timer(MODE_UPDATE_RATE, self.timer_cb)
        self.pin = getPin(MODE)
        pinMode(self.pin, IOType.INPUT)
    
    def timer_cb(self):
        mode = digitalRead(self.pin) 
        mode = not mode # hardware hack
        msg = Mode()
        msg.auto_mode = mode
        self.pub.publish(msg) 

def main():
    rclpy.init()
    node = ModeNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()