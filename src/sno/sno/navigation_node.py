import math
import time
from typing import Tuple
import rclpy
import rclpy.node
from sno_interfaces.msg import Waypoints, Coord, Motors, Mode, Heading, FlutterSchedule
from sno.lib.config import *


class NavigationNode(rclpy.node.Node):
    heading = None
    location = None
    waypoints = None
    curr_waypoint = 0

    def __init__(self):
        super().__init__('navigation_node')
        self.log = self.get_logger().info
        self.create_subscription(Mode, '/mode', self.mode_cb, 10)
        self.create_subscription(Waypoints, '/waypoints', self.waypoints_cb, 10)
        self.create_subscription(Heading, '/heading', self.heading_cb, 10)
        self.create_subscription(Coord, '/location', self.location_cb, 10)
        self.create_subscription(FlutterSchedule, '/schedule',self.schedule_cb, 10)
        self.motor_pub = self.create_publisher(Motors, '/motors', 1)
        self.clear_time = time.time() - 10 # 10 seconds ago
    
    def schedule_cb(self, data:FlutterSchedule):
        self.clear_time = data.time
    
    def mode_cb(self, data:Mode):
        self.auto_mode = data.auto
    
    def heading_cb(self,data:Heading):
        self.heading = data.heading
    
    def location_cb(self,data:Coord):
        self.location = data
    
    def waypoints_cb(self,data:Waypoints):
        self.waypoints = data.waypoints
    
    def motion_calc(self) -> Tuple(float, float):
        # stop if at global goal
        global_goal = self.waypoints[-1]
        square_dist_from_global = ((global_goal.lat - self.location.lat)**2 + (global_goal.lon - self.location.lon)**2) / GPS_TO_IN
        if square_dist_from_global < WAYPOINT_TOLERANCE_IN**2:
            self.log('at goal, waiting   (NavigationNode.plan)')
            return 0, 0

        local_goal = self.waypoints[self.curr_waypoint]        
        # next local point
        square_dist_from_local = ((local_goal.lat - self.location.lat)**2 + (local_goal.lon - self.location.lon)**2) / GPS_TO_IN
        if square_dist_from_local < WAYPOINT_TOLERANCE_IN**2:
            self.curr_waypoint += 1
            self.log(f'next waypoint {self.curr_waypoint}/{len(self.waypoints)}   (NavigationNode.plan)')
            return 0, 0
        
        # move towards local_goal
        desired_heading = math.tan2(local_goal.lat - self.location.lat, local_goal.lon - self.location.lon)
        ang = (desired_heading - self.heading + math.pi) % (2 * math.pi) - math.pi

        linear = DEFAULT_LINEAR_VEL_FPS * min(1, square_dist_from_local / DIST_TO_START_SCALING_LIN_VEL_IN**2)
        angular = DEFAULT_ANGULAR_VEL_RPS * min(1, ang / ANGLE_TO_START_SCALING_ANG_VEL_RAD)
        return linear, angular 

    def move(self, linear, angular):
        ''' @linear: linear speed in ft/s
            @angular: angular speed in rad/s
        '''
        left_speed = linear - angular * (ROBOT_WIDTH_IN/12) / 2
        right_speed = linear + angular * (ROBOT_WIDTH_IN/12) / 2

        left = left_speed / SPEED_AT_FULL_MOTOR_FPS
        right = right_speed / SPEED_AT_FULL_MOTOR_FPS

        left = max(min(1, left), -1)
        right = max(min(1, right), -1)

        msg = Motors()
        msg.left = left
        msg.right = right
        msg.auger = AUGER_ON_IN_AUTO
        msg.enable = [True,True,AUGER_ON_IN_AUTO]
        self.motor_pub.publish(msg)
    
    def run(self):
        while rclpy.ok():
            if self.auto_mode:
                if time.time() > self.clear_time:
                    linear, angular = self.motion_calc()
                    self.move(linear, angular)
                else:
                    self.move(0, 0)


    

        


def main():
    rclpy.init()
    node = NavigationNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()