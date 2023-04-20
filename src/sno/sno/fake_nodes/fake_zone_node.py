import rclpy
import rclpy.node
from sno_interfaces.msg import FlutterZones

class FakeZoneNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('fake_zone_node')
        self.pub = self.create_publisher(FlutterZones, '/flutter_zones', 1)
        self.create_timer(1,self.timer_cb)

    def timer_cb(self):
        msg = FlutterZones()
        msg.clear_zone[0].lat = 40.433307
        msg.clear_zone[0].lon = -86.914142
        msg.clear_zone[1].lat = 40.433276
        msg.clear_zone[1].lon = -86.914045
        msg.snow_zone[0].lat = 40.433273
        msg.snow_zone[0].lon = -86.914140
        msg.snow_zone[1].lat = 40.433264 
        msg.snow_zone[1].lon = -86.914060
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeZoneNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()