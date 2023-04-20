import math
import rclpy
import rclpy.node
# import matplotlib
# matplotlib.use('TkAgg')
# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
from sno.lib.stepper import StepperMotor, MotorInterfaceType
from sno_interfaces.msg import FlutterZones, Coord, Mode, Stepper, Heading
from sno.lib.config import STEPPER_DIR, STEPPER_STEP, STEPPER_INCREMENT, STEPPER_MAX_STEPS

class Loc():
    lat: float
    lon: float
    def __init__(self, lat:float, lon:float) -> None:
        self.lat = lat
        self.lon = lon

# Stepper must be pointed straight forward at power on
class StepperNode(rclpy.node.Node):
    robot_loc:Loc|None = None
    zone_point_loc:Loc|None = None
    robot_heading:float|None = None
    
    def __init__(self, debug=False, vis=False):
        super().__init__('stepper_node')
        self.log = self.get_logger().info
        # self.vis = vis
        # self.debug = debug
        # if self.vis:
        #     self.create_timer(0.1, self.update_graph_callback)
        #     self.create_display()
        
        self.desired_abs_heading = 0 

        # get positions
        self.create_subscription(Coord, '/location', self.gps_cb, 10)
        self.create_subscription(Heading, '/heading', self.heading_cb, 10)
        self.create_subscription(FlutterZones, '/flutter_zones', self.zone_cb, 10)
        self.create_subscription(Mode, '/mode', self.mode_cb, 10)
        self.create_subscription(Stepper, '/stepper', self.stepper_cb, 10)
        self.create_timer(0.0001, self.stepper_run_cb)
        
        self.steps_per_revolution = 200         
        self.stepper = StepperMotor(MotorInterfaceType.DRIVER, pin1=STEPPER_DIR, pin2=STEPPER_STEP)
        self.stepper.setMaxSpeed(500)
        self.stepper.setAcceleration(50)
        
        self.auto_mode = False

    def stepper_cb(self, data:Stepper):
        if not self.auto_mode:
            self.stepper.setSpeed(0)
            if data.right and self.stepper.currentPosition() < STEPPER_MAX_STEPS:
                    self.stepper.setSpeed(100)
            elif data.left and -STEPPER_MAX_STEPS < self.stepper.currentPosition():
                    self.stepper.setSpeed(-100)
    
    def mode_cb(self, data:Mode):
        self.auto_mode = data.auto_mode
    
    def stepper_run_cb(self):
        self.stepper.runSpeed()

    # def update_graph_callback(self):
    #     self.update_display()

    def gps_cb(self,data:Coord):
        self.robot_loc = Loc(data.lat, data.lon)
        self.recalculate() 
    
    def zone_cb(self,data:FlutterZones):
        self.zone = data.snow_zone
        lon,lat = self.closest_point_on_rect(self.robot_loc, self.zone[0], self.zone[1])
        self.zone_point_loc = Loc(lat,lon)
        self.recalculate()
    
    def heading_cb(self, data:Heading):
        self.heading = data.heading
        self.recalculate()
    
    def steps_to_radians(self,steps):
        return steps / self.steps_per_revolution * 2*math.pi
    def radians_to_steps(self, radians):
        return radians / (2*math.pi) * self.steps_per_revolution
    def recalculate(self):
        if not self.auto_mode:
            return
        if self.robot_loc is None or self.zone_point_loc is None or self.heading is None:
            return
        self.desired_abs_heading = math.atan2(self.zone_point_loc.lon - self.robot_loc.lon, self.zone_point_loc.lat - self.robot_loc.lat)
        desired_rel_heading = self.desired_abs_heading - self.heading - self.steps_to_radians(self.stepper._currentPos)
        desired_rel_heading = (desired_rel_heading + math.pi) % (2 * math.pi) - math.pi # keeps from flipping -179.99 to 179.99 at 180deg boundary
        steps = self.radians_to_steps(desired_rel_heading)
        self.stepper.move(steps)

    def closest_point_on_rect(self, point:Coord, corner1:Coord, corner2:Coord):
        point = (point.lon, point.lat)
        corner1 = (corner1.lon, corner1.lat)
        corner2 = (corner2.lon, corner2.lat)
        x1, y1 = corner1
        x2, y2 = corner2

        # find the closest point on each of the four lines that make up the rectangle
        p,d = 4*[0], 4*[0]
        p[0],d[0] = self.closest_point_on_line(point, corner1, (x2, y1))
        p[1],d[1] = self.closest_point_on_line(point, (x2, y1), corner2)
        p[2],d[2] = self.closest_point_on_line(point, corner2, (x1, y2))
        p[3],d[3] = self.closest_point_on_line(point, (x1, y2), corner1)
        closest_point = p[d.index(min(d))]
        return closest_point

    def closest_point_on_line(self,point, line_start, line_end):
        x0, y0 = point
        x1, y1 = line_start
        x2, y2 = line_end

        # find the closest point on the line that contains the segment
        u = ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / ((x2 - x1) ** 2 + (y2 - y1) ** 2)
        closest_point = (x1 + u * (x2 - x1), y1 + u * (y2 - y1))
        # check if the closest point is on the segment
        if u < 0:       # closest point is before the start of the segment
            closest_point = line_start
        elif u > 1:     # closest point is after the end of the segment
            closest_point = line_end
        
        distance_squared = (closest_point[0]-x0)**2 + (closest_point[1]-x1)**2
        return closest_point, distance_squared

    
    # # visualization for validation
    # def create_display(self):
    #     plt.ion()
    #     self.fig, self.ax = plt.subplots()
    #     self.robot_loc = self.ax.scatter([], [], color='b')
    #     self.robot_heading_ar = self.ax.arrow(0,0,0,0)
    #     self.chute_heading_ar = self.ax.arrow(0,0,0,0)
    #     # self.desired_heading_ar = self.ax.arrow(0,0,0,0)
    #     self.zone_rect = patches.Rectangle((0,0),1,1,facecolor='r',alpha=0.3)
    #     self.ax.add_patch(self.zone_rect)
    
    # def update_display(self):
    #     if self.robot_loc is not None and self.zone_point_loc is not None and self.heading is not None:
    #         # remove old
    #         self.robot_heading_ar.remove()
    #         self.chute_heading_ar.remove()
    #         # self.desired_heading_ar.remove()

    #         # # desired heading - blue
    #         # length = 0.000004
    #         width = 0.0000002
    #         # self.desired_heading_ar = self.ax.arrow(self.robot_loc.lon, self.robot_loc.lat, length*math.sin(self.desired_abs_heading), length*math.cos(self.desired_abs_heading), width=width, color='b')
            
    #         # current robot heading - green
    #         length = 0.000003
    #         self.robot_heading_ar = self.ax.arrow(self.robot_loc.lon, self.robot_loc.lat, length*math.sin(self.heading), length*math.cos(self.heading), width=width, color='g')
            
    #         # current chute heading - red
    #         length = 0.000002
    #         current_heading = self.steps_to_radians(self.stepper._currentPos) + self.heading 
    #         self.chute_heading_ar = self.ax.arrow(self.robot_loc.lon, self.robot_loc.lat, length*math.sin(current_heading), length*math.cos(current_heading), width=width, color='r')

    #         # # zone locationr
    #         self.zone_rect.set_xy((self.zone[0].lon, self.zone[0].lat))
    #         self.zone_rect.set_width(self.zone[1].lon - self.zone[0].lon)
    #         self.zone_rect.set_height(self.zone[1].lat - self.zone[0].lat)

    #     self.ax.relim()
    #     self.ax.autoscale_view()
    #     self.fig.canvas.draw()
    #     self.fig.canvas.flush_events()

def main():
    rclpy.init()
    node = StepperNode(debug=False, vis=False)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
