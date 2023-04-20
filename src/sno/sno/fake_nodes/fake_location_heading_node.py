import rclpy
import rclpy.node
from sno_interfaces.msg import Coord
from std_msgs.msg import Float64
import math

class FakeLocationHeadingNode(rclpy.node.Node):
    lin_rate = 0.01
    lin_vel = 1/8 * (1/364000) # 1ft/s
    head_rate = 0.01
    head_vel =  0 * (2*math.pi / 360) # 1 deg/s
    lon = -86.914142 + 0.00004
    lat = 40.433307 - 0.00003
    head = math.pi/2

    def __init__(self):
        super().__init__('fake_location_heading_node')
        self.log = self.get_logger().info
        self.location_pub = self.create_publisher(Coord, '/location', 1)
        self.heading_pub = self.create_publisher(Float64, '/heading', 1)
        self.create_timer(self.lin_rate, self.location_cb)
        self.create_timer(self.head_rate, self.heading_cb)
    
    def location_cb(self):
        self.lon += self.lin_vel*self.lin_rate * math.sin(self.head)
        self.lat += self.lin_vel*self.lin_rate * math.cos(self.head)
        msg = Coord()
        msg.lat = self.lat
        msg.lon = self.lon
        self.location_pub.publish(msg)
    
    def heading_cb(self):
        self.head += self.head_vel*self.head_rate
        msg = Float64()
        msg.data = self.head
        self.heading_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLocationHeadingNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()