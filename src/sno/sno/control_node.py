import rclpy
import rclpy.node
from sno_interfaces.msg import FlutterControl, Mode, Motors, Stepper
from sno.lib.config import DEFAULT_SPEED_PERCENT

class ControlNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('control_node')
        self.log = self.get_logger().info
        self.create_subscription(FlutterControl, '/flutter_control', self.control_cb, 10)
        self.create_subscription(Mode, '/mode', self.mode_cb, 10)
        self.motor_pub = self.create_publisher(Motors, '/motors', 1)
        self.stepper_pub = self.create_publisher(Stepper, '/stepper', 1)
        self.auto_mode = False
        self.on = False
        self.auger_on = False

    def mode_cb(self, data:Mode):
        self.auto_mode = data.auto_mode
    
    def control_cb(self, data:FlutterControl):
        left,right,forward,backward,on,chute_left,chute_right,auger_on = [0,1,2,3,4,5,6,7]
        if data.enable[on]:
            self.on = data.on
        
        l_en, r_en = False, False
        if data.enable[left] or data.enable[forward] or data.enable[backward]:
            l_en = True
        if data.enable[right] or data.enable[forward] or data.enable[backward]:
            r_en = True
        
        l = r = 0
        if data.enable[forward] and data.forward:
            l = r = DEFAULT_SPEED_PERCENT
        elif data.enable[backward] and data.backward:
            l = r = -DEFAULT_SPEED_PERCENT
        elif data.enable[left] and data.left:
            l = -DEFAULT_SPEED_PERCENT
            r = DEFAULT_SPEED_PERCENT
        elif data.enable[right] and data.right:
            l = DEFAULT_SPEED_PERCENT
            r = -DEFAULT_SPEED_PERCENT
        
        if not self.on:
            l_en = r_en = True
            l = r = 0

        msg = Motors()
        msg.left = float(l) if self.on else 0.0
        msg.right = float(r) if self.on else 0.0
        msg.auger = data.auger_on if data.enable[auger_on] else False
        msg.enable = [l_en, r_en, data.enable[auger_on]]
        self.motor_pub.publish(msg)
        
        msg = Stepper()
        if data.enable[chute_left]:
            msg.left = data._chute_left
        if data.enable[chute_right]:
            msg.right = data.chute_right
        self.stepper_pub.publish(msg)

def main():
    rclpy.init()
    node = ControlNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()