#!/usr/bin/env python3

import time
import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import rclpy
from rclpy.node import Node
from sno_interfaces.msg import Heading
from sno.lib.config import COMPASS_UPDATE_RATE

import busio
from adafruit_blinka.microcontroller.generic_linux.libgpiod_pin import Pin

class CompassNode(Node):
    def __init__(self):
        super().__init__('compass_node')
        self.pub = self.create_publisher(Heading, '/heading', 10)

        # self.i2c = busio.I2C(Pin((0, 4)), Pin((0, 5)))
        self.i2c = board.I2C() # uses board and board.SDA
        self.accel = adafruit_lsm303_accel.LSM303_Accel(self.i2c)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)
        self.create_timer(COMPASS_UPDATE_RATE, self.timer_cb)

    def timer_cb(self):
        acc_x, acc_y, acc_z = self.accel.acceleration
        mag_x, mag_y, mag_z = self.mag.magnetic

        # Compute orientation
        roll, pitch = self.compute_orientation(acc_x, acc_y, acc_z)
        heading = self.compute_heading(mag_x, mag_y, mag_z, roll, pitch)

        msg = Heading()
        msg.heading = heading
        self.log(f'{heading}')
        self.pub.publish(msg)

    def compute_orientation(self, ax, ay, az):
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        return roll, pitch

    def compute_heading(self, mx, my, mz, roll, pitch):
        # Tilt compensation
        bx = mx * math.cos(pitch) + my * math.sin(roll) * math.sin(pitch) - mz * math.cos(roll) * math.sin(pitch)
        by = my * math.cos(roll) + mz * math.sin(roll)

        # Compute heading
        heading = math.atan2(by, bx)
        if heading < 0:
            heading += 2 * math.pi
        return heading

def main(args=None):
    rclpy.init(args=args)
    node = CompassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
