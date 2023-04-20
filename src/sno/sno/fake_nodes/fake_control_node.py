import rclpy
import rclpy.node

from sno_interfaces.msg import FlutterControl
from sno.lib.msgs import setMsgAttr

class FakeControlNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('fake_control_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(FlutterControl, '/control', 1)
        self.create_timer(0.5, self.timer_cb)
        self.create_timer(5, self.timer_cb2)
    
    def timer_cb(self):
        msg = FlutterControl()
        setMsgAttr(msg, 'chute_right', True)
        en = [False]*8
        en[6] = True
        setMsgAttr(msg, 'enable', en)
        self.pub.publish(msg) 
    
    def timer_cb2(self):
        msg = FlutterControl()
        setMsgAttr(msg, 'on', True)
        en = [False]*8
        en[4] = True
        setMsgAttr(msg, 'enable', en)
        self.pub.publish(msg) 


def main():
    rclpy.init()
    node = FakeControlNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()