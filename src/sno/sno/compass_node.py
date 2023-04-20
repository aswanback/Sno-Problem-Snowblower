#!/usr/bin/env python3
import time
import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sno_interfaces.msg import Heading
from sno.lib.config import COMPASS_UPDATE_RATE
import busio
from adafruit_blinka.microcontroller.generic_linux.libgpiod_pin import Pin

class CompassNode(Node):
    class xyz:
        def __init__(self, x=None, y=None, z=None):
            self.x = x
            self.y = y
            self.z = z
    window = []
    window_size = 50
    n = 32767
    m_min = xyz(-n,-n,-n)
    m_max = xyz(n,n,n)
    
    def __init__(self):
        super().__init__('compass_node')
        self.log = self.get_logger().info
        self.pub = self.create_publisher(Heading, '/heading', 10)
        # self.i2c = board.I2C() # uses board and board.SDA
        self.i2c = busio.I2C((1,76), (1,75))
        self.accel = adafruit_lsm303_accel.LSM303_Accel(self.i2c)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
        self.create_timer(COMPASS_UPDATE_RATE, self.timer_cb)

    def timer_cb(self):
        # acc_x, acc_y, acc_z = self.accel.acceleration
        # mag_x, mag_y, mag_z = self.mag.magnetic
        # # Compute orientation
        # roll, pitch = self.compute_orientation(acc_x, acc_y, acc_z)
        # heading = self.compute_heading(mag_x, mag_y, mag_z, roll, pitch)
        acc = self.xyz(*self.accel.acceleration)
        mag = self.xyz(*self.mag.magnetic)
        
        heading = self.heading(mag, acc)
        heading = self.moving_average_filter(heading)

        msg = Heading()
        msg.heading = heading
        self.log(f'{heading}')
        self.pub.publish(msg)

    # def compute_orientation(self, ax, ay, az):
    #     roll = math.atan2(ay, az)
    #     pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
    #     return roll, pitch

    # def compute_heading(self, mx, my, mz, roll, pitch):
    #     # Tilt compensation
    #     bx = mx * math.cos(pitch) + my * math.sin(roll) * math.sin(pitch) - mz * math.cos(roll) * math.sin(pitch)
    #     by = my * math.cos(roll) + mz * math.sin(roll)

    #     # Compute heading
    #     heading = math.atan2(by, bx)
    #     if heading < 0:
    #         heading += 2 * math.pi
    #     # heading = (heading + math.pi) % (2 * math.pi) - math.pi
    #     heading *= 180 / math.pi
    #     return heading

    def heading(self, mag:xyz, acc:xyz, from_vector=xyz(0,-1,0)) -> float:

        temp_m = [mag.x, mag.y, mag.z]
        # subtract offset (average of min and max) from magnetometer readings
        temp_m[0] -= (self.m_min.x + self.m_max.x) / 2
        temp_m[1] -= (self.m_min.y + self.m_max.y) / 2
        temp_m[2] -= (self.m_min.z + self.m_max.z) / 2
        temp_m = self.xyz(*temp_m)

        # compute E and N
        E = self.vector_cross(temp_m, acc)
        E = self.vector_normalize(E)
        N = self.vector_cross(acc, E)
        N = self.vector_normalize(N)

        # compute heading
        heading = math.atan2(self.vector_dot(E, from_vector), self.vector_dot(N, from_vector)) * 180 / math.pi
        if heading < 0: heading += 360
        return heading

    def vector_cross(self,a:xyz, b:xyz):
        out = self.xyz()
        out.x = (a.y * b.z) - (a.z * b.y)
        out.y = (a.z * b.x) - (a.x * b.z)
        out.z = (a.x * b.y) - (a.y * b.x)
        return out
        
    def vector_dot(self,a:xyz, b:xyz):
        return (a.x * b.x) + (a.y * b.y) + (a.z * b.z)
    
    def vector_normalize(self,a:xyz):
        magn = math.sqrt(self.vector_dot(a,a))
        return self.xyz(a.x/magn, a.y/magn, a.z/magn)

    def moving_average_filter(self, data):
        # Append the new data point to the window
        self.window.append(data)
        # If the window is full, remove the oldest data point
        if len(self.window) > self.window_size:
            self.window.pop(0)
        # Calculate the moving average over the window
        return np.mean(self.window)

def main(args=None):
    rclpy.init(args=args)
    node = CompassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
