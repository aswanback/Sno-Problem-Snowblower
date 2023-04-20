import time
import rclpy
import rclpy.node
from sno.lib.arduino import digitalWrite, digitalRead, getPin, IOType, pinMode
from sno.lib.config import *
from sno_interfaces.msg import Ultrasonics

class UltrasonicNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(Ultrasonics, '/ultrasonic_stop', 1)
        self.create_timer(ULTRASONIC_PULSE_RATE, self.timer_cb)

        self.trig, self.echo = [None]*2, [None]*2
        self.trig[0], self.echo[0], self.trig[1], self.echo[1] = [getPin(p) for p in [US_TRIG_1, US_ECHO_1, US_TRIG_2, US_ECHO_2]]
        pinMode(self.trig[0], IOType.OUTPUT)
        pinMode(self.echo[0], IOType.INPUT)
        pinMode(self.trig[1], IOType.OUTPUT)
        pinMode(self.echo[1], IOType.INPUT)
        
    
    def timer_cb(self):
        dist1 = self.distance(self.trig[0], self.echo[0])
        dist2 = self.distance(self.trig[1], self.echo[1])
        self.log(f'{dist1}, {dist2}')
        stop = False
        if dist1 < US_SAFE_MIN_CM or dist2 < US_SAFE_MIN_CM:
            stop = True
        msg = Ultrasonics()
        msg.stop = stop
        self.pub.publish(stop)

    def distance(self, trig, echo):
        digitalWrite(trig, 1)
        time.sleep(0.00001)
        digitalWrite(trig, 0)

        StartTime = time.time_ns()
        StopTime = time.time_ns()

        while digitalRead(echo) == 0:
            StartTime = time.time_ns()
        while digitalRead(echo) == 1:
            StopTime = time.time_ns()

        TimeElapsed = (StopTime - StartTime) / 1e6
        distance = (TimeElapsed * 34300) / 2
        return distance

def main():
    rclpy.init()
    node = UltrasonicNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()