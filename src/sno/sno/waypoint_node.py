
import rclpy
import rclpy.node
from sno_interfaces.msg import FlutterZones, Coord, Waypoints
import math
from geopy.distance import distance
from sno.lib.config import ROBOT_LENGTH_IN, ROBOT_WIDTH_IN, PATH_OVERLAP_IN
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

class WaypointNode(rclpy.node.Node):
    def __init__(self, vis=False):
        super().__init__('waypoint_node')
        self.vis = vis
        self.log = self.get_logger().info
        self.waypoints = None
        self.zones = None

        self.create_subscription(FlutterZones, '/flutter_zones', self.zones_cb, 10)
        self.pub = self.create_publisher(Waypoints, '/waypoints', 1)
        if self.vis: 
            self.createPlot()
    
    def zones_cb(self,data):
        self.zone = data.clear_zone
        pattern = self.generate_coverage_pattern(data.clear_zone[0], data.clear_zone[1], ROBOT_WIDTH_IN, PATH_OVERLAP_IN)
        msg = Waypoints()
        msg.waypoints = pattern
        self.waypoints = msg
        self.pub.publish(msg)
        if self.vis:
            self.update()

    def generate_coverage_pattern(self, corner1, corner2, robot_width_in, overlap_in):
        latitudes = [corner1.lat, corner2.lat]
        longitudes = [corner1.lon, corner2.lon]
        min_lat, max_lat = min(latitudes), max(latitudes)
        min_lon, max_lon = min(longitudes), max(longitudes)

        lat_diff = (max_lat - min_lat) * 364000 #distance((min_lat, min_lon), (max_lat, min_lon)).ft
        lon_diff = (max_lon - min_lon) * 364000 #distance((min_lat, min_lon), (min_lat, max_lon)).ft
        robot_width = robot_width_in / 12
        overlap = overlap_in / 12

        num_rows = math.ceil((lat_diff - robot_width) / (robot_width - overlap))
        pattern = []
        for i in range(num_rows+1):
            lat = min_lat + i * (robot_width - overlap) / (364000) + robot_width / 2 / (364000)
            if i % 2 == 0:
                const1 = 0
                const2 = 1
            else:
                const1 = 1
                const2 = 0
            lon1 = min_lon + const1 * lon_diff / 364000
            lon2 = min_lon + const2 * lon_diff / 364000
            msg = Coord()
            msg.lat = lat
            msg.lon = lon1
            pattern.append(msg)
            msg = Coord()
            msg.lat = lat
            msg.lon = lon2
            pattern.append(msg)
        if len(pattern) > 0: pattern.append(pattern[0])
        return pattern

    def createPlot(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], color='b')
        self.pts = self.ax.scatter([], [], c='r')
        self.zone_rect = patches.Rectangle((0,0),1,1,facecolor='g',alpha=0.2)
        self.ax.add_patch(self.zone_rect)
    
    def update(self):
        if self.waypoints is None or len(self.waypoints.waypoints) == 0: return
        x = [w.lon for w in self.waypoints.waypoints]
        y = [w.lat for w in self.waypoints.waypoints]
        self.line.set_data(x, y)
        self.pts.set_offsets(np.column_stack((x, y)))

        self.zone_rect.set_xy((self.zone[0].lon, self.zone[0].lat))
        self.zone_rect.set_width(self.zone[1].lon - self.zone[0].lon)
        self.zone_rect.set_height(self.zone[1].lat - self.zone[0].lat)
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    

def main():
    rclpy.init()
    node = WaypointNode(vis=True)
    rclpy.spin(node)

if __name__ == '__main__':
    main()