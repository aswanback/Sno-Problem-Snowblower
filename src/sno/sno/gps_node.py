import re
import subprocess
import rclpy
import rclpy.node
from sno.lib.config import GPS_UPDATE_RATE
from sno_interfaces.msg import Coord
import serial
import pynmea2

class GPSNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('gps_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(Coord, '/location', 1)
        self.serial_port = serial.Serial('/dev/ttyACM0',9600, timeout=5.0) # '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 9600, timeout=5.0)
        self.create_timer(GPS_UPDATE_RATE, self.timer_cb)
        # self.proc = subprocess.Popen('str2str -in ntrip://purdue121:purdue121@158.59.49.226:7071/RTCM3_INWL#rtcm3 -out serial://ttyUSB0:9600#rtcm3',shell=True)
        self.proc = subprocess.Popen('str2str -in ntrip://abrown:RCh5tXgz@rtgpsout.unavco.org:2101/HDIL_RTCM3#rtcm3 -out serial://ttyUSB0:9600#rtcm3',shell=True)
        
        self.lats, self.lons = [], []
        self.create_timer(60, self.print_cb)

    def timer_cb(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if line.startswith('$GPGGA'):
            try:
                nmea = pynmea2.parse(line)
                if nmea.latitude and nmea.longitude:
                    msg = Coord()
                    msg.lat = nmea.latitude
                    msg.lon = nmea.longitude
                    self.lats.append(nmea.latitude)
                    self.lons.append(nmea.longitude)
                    self.log(f'{nmea.latitude}, {nmea.longitude}')
                    self.pub.publish(msg)
            except pynmea2.ParseError:
                self.log('GPSNode: ParseError')
    
    def print_cb(self):
        self.log(f'{self.lats}')
        self.log(f'{self.lons}')

    def __del__(self):
        self.proc.terminate()

def main():
    rclpy.init()
    node = GPSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()