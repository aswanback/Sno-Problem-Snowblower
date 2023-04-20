#!/usr/bin/env python3
import board
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import smbus2

class Compass:
    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self.address = 0x1e  # LSM303 magnetometer address
        # Configure the magnetometer
        self.bus.write_byte_data(self.address, 0x00, 0x14)  # set the magnetic field range to +/- 1.3 Gauss
        self.bus.write_byte_data(self.address, 0x01, 0x20) 
        # configure accel
        CTRL_REG1_A = 0x20
        self.bus.write_byte_data(self.address, CTRL_REG1_A, 0x57)

    def read_mag(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = (data[0] << 8) | data[1]
        y = (data[2] << 8) | data[3]
        z = (data[4] << 8) | data[5]
        return x,y,z

    def read_acc(self):
        OUT_X_L_A = 0x28
        OUT_Y_L_A = 0x2A
        OUT_Z_L_A = 0x2C
        x = self.bus.read_word_data(self.address, OUT_X_L_A)
        y = self.bus.read_word_data(self.address, OUT_Y_L_A)
        z = self.bus.read_word_data(self.address, OUT_Z_L_A)

        # Convert to g values
        scale = 2.0 / 32768.0
        x_g = x * scale
        y_g = y * scale
        z_g = z * scale
        return x_g, y_g, z_g

    def get_heading(self):
        acc_x, acc_y, acc_z = self.read_acc()
        mag_x, mag_y, mag_z = self.read_mag()

        roll, pitch = self.compute_orientation(acc_x, acc_y, acc_z)
        return self.compute_heading(mag_x, mag_y, mag_z, roll, pitch)

    def compute_orientation(self, ax, ay, az):
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        return roll, pitch

    def compute_heading(self, mx, my, mz, roll, pitch):
        # Tilt compensation
        bx = roll # mx * math.cos(pitch) + my * math.sin(roll) * math.sin(pitch) - mz * math.cos(roll) * math.sin(pitch)
        by = pitch # my * math.cos(roll) + mz * math.sin(roll)

        # Compute heading
        heading = math.atan2(by, bx)
        if heading < 0:
            heading += 2 * math.pi
        return heading

if __name__ == '__main__':
    c = Compass()
    while True:
        print(c.read_mag())