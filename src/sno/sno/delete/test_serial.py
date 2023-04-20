import serial
ser = serial.Serial()
ser.baud = 9600
ser.port = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
ser.open()
ser.write(str.encode('test'))


# str2str -in ntrip://abrown:RCh5tXgz@rtgpsout.unavco.org:2101/HDIL_RTCM3 -out serial://ttyUSB0
# str2str -in ntrip://abrown:RCh5tXgz@rtgpsout.unavco.org:2101/HDIL_RTCM3#rtcm3 -out serial://ttyUSB0:9600#rtcm3
# str2str -in ntrip://purdue121:purdue121@158.59.49.226:7071/RTCM3_INWL#rtcm3 -out serial://ttyUSB0:9600#rtcm3

