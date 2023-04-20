import re
import subprocess
import rclpy
import rclpy.node
from sno.lib.config import GPS_UPDATE_RATE
from sno_interfaces.msg import Coord, Rtk
import serial
import pynmea2
from filterpy.kalman import KalmanFilter
import numpy as np



class GPSNode(rclpy.node.Node):
    window = []
    window_size = 10
    def __init__(self):
        super().__init__('gps_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(Coord, '/location', 1)
        self.serial_port = serial.Serial('/dev/ttyACM0',9600, timeout=5.0) # '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, timeout=5.0)
        self.create_timer(GPS_UPDATE_RATE, self.timer_cb)
        self.lats, self.lons = [], []
        self.create_timer(5, self.save_cb)

        # self.proc = subprocess.Popen('str2str -in ntrip://purdue121:purdue121@158.59.49.226:7071/RTCM3_INWL#rtcm3 -out serial://ttyUSB0:9600#rtcm3',shell=True)
        # self.proc = subprocess.Popen('str2str -in ntrip://abrown:RCh5tXgz@rtgpsout.unavco.org:2101/HDIL_RTCM3#rtcm3 -out serial://ttyUSB0:9600#rtcm3',shell=True)
        self.execute_bash()
    
    def execute_bash(self):
        cmd = ['str2str', '-in', 'ntrip://abrown:RCh5tXgz@rtgpsout.unavco.org:2101/HDIL_RTCM3#rtcm3', '-out', 'serial://ttyUSB0:9600#rtcm3']
        rtkpub = self.create_publisher(Rtk, '/rtk', 1)
        popen = subprocess.Popen(cmd, stderr=subprocess.PIPE, universal_newlines=True)
        for stdout_line in iter(popen.stderr.readline, ""):
            msg = Rtk()
            msg.data = stdout_line
            rtkpub.publish(msg)
        popen.stdout.close()

    def timer_cb(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if line.startswith('$GNGGA'):
            try:
                nmea = pynmea2.parse(line)
                if nmea.latitude and nmea.longitude:
                    self.moving_average_filter((nmea.latitude,nmea.longitude))
                    
                    msg = Coord()
                    msg.lat = nmea.latitude
                    msg.lon = nmea.longitude
                    self.lats.append(nmea.latitude)
                    self.lons.append(nmea.longitude)
                    self.pub.publish(msg)
            except pynmea2.ParseError:
                self.log('GPSNode: ParseError')
    
    def moving_average_filter(self, data):
        # Append the new data point to the window
        self.window.append(data)
        # If the window is full, remove the oldest data point
        if len(self.window) > self.window_size:
            self.window.pop(0)
        # Calculate the moving average over the window
        return np.mean(self.window)

    
    def save_cb(self):
        with open('/home/pi/ws/src/sno/sno/lib/gps_data.txt','w') as f:
            f.write(str(self.lats))
            f.write('\n')
            f.write(str(self.lons))
            f.write('\n')

def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()