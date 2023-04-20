import rclpy
import rclpy.node as node
from sensor_msgs.msg import NavSatFix

class FakeUINode(rclpy.node.Node):
    def __init__(self, debug=False):
        super().__init__('fake_ui_node')
        self.log = self.get_logger().info
        self.debug = debug
        self.pub = self.create_publisher(NavSatFix, '/zone_pos', 1)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.latitude = 40.433261
        self.longitude = -86.913871

    def timer_callback(self):
        incr = 0 #0.00001
        gps = NavSatFix()
        self.latitude += incr
        self.longitude += incr
        gps.latitude = self.latitude
        gps.longitude = self.longitude
        self.pub.publish(gps)
        if self.debug:
            self.log(f'at: {gps.longitude}, {gps.latitude}')


def main():
    rclpy.init()
    node = FakeUINode(debug=False)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
