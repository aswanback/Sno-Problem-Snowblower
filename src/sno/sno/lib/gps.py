
from sno.lib.arduino import digitalWrite, pinMode

import serial
ser = serial.Serial()
ser.baud = 9600
ser.port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
ser.open()

while True:
    s = ser.read_all()
    print(s)