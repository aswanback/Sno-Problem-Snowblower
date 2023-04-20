import rclpy
import rclpy.node as node
import math
from sensor_msgs.msg import NavSatFix
import random 

class FakeGPSNode(rclpy.node.Node):
    last_gps = None
    lat_incr = 0.000001
    long_incr = 0.000001
    angle = 0

    def __init__(self, debug=False):
        super().__init__('fake_gps_node')
        self.log = self.get_logger().info
        self.debug = debug
        self.pub = self.create_publisher(NavSatFix, 'global_position/raw/fix', 1)
        self.timer = self.create_timer(0.05, self.timer_callback_circle)
        self.last_gps = NavSatFix()
        self.last_gps.latitude = 40.433261
        self.last_gps.longitude = -86.913871

    def timer_callback_line(self):
        gps = NavSatFix()
        gps.latitude = self.last_gps.latitude + self.lat_incr
        gps.longitude = self.last_gps.longitude + self.long_incr
        self.pub.publish(gps)
        self.last_gps = gps
        if self.debug:
            self.log(f'gps: {gps.longitude}, {gps.latitude}')
    
    def timer_callback_circle(self):
        steps = 300
        radius = 0.01
        sign = 1 # -1 if random.random() < 0.2 else 1

        gps = NavSatFix()
        gps.latitude = self.last_gps.latitude +  radius*math.sin(self.angle)
        gps.longitude = self.last_gps.longitude + 0.5*radius*math.cos(self.angle)
        self.pub.publish(gps)
        self.angle += sign*2*math.pi / steps
        if self.debug:
            self.log(f'gps: {gps.longitude}, {gps.latitude}')


def main():
    rclpy.init()
    node = FakeGPSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
