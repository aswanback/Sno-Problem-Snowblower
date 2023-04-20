import asyncio
import rclpy
import rclpy.node
from sno_interfaces.msg import FlutterZones, FlutterControl, FlutterSchedule
import websockets
import json
import numpy as np
from sno.lib.msgs import setMsgAttr, getMsgAttr

class FlutterNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('flutter_node')
        self.log = self.get_logger().info
        self.zone_pub = self.create_publisher(FlutterZones, '/flutter_zones', 1)
        self.control_pub = self.create_publisher(FlutterControl, '/flutter_control', 1)
        self.schedule_pub = self.create_publisher(FlutterSchedule, '/flutter_schedule', 1)

        

    def get_enable(self,ls):
        return [True if l is not None else False for l in ls ]
    
    def make_control(self, d):
        field = list(d.keys())[0]
        value = d[field]
        c_msg = FlutterControl()

        if field == 'left':
            c_msg.left = value
            c_msg.enable[0] = True
        elif field == 'right':
            c_msg.right = value
            c_msg.enable[1] = True
        elif field == 'forward':
            c_msg.forward = value
            c_msg.enable[2] = True
        elif field == 'backward':
            c_msg.backward = value
            c_msg.enable[3] = True
        elif field == 'on':
            c_msg.on = value
            c_msg.enable[4] = True
        elif field == 'chute_left':
            c_msg.chute_left = value
            c_msg.enable[5] = True
        elif field == 'chute_right':
            c_msg.chute_right = value
            c_msg.enable[6] = True
        elif field == 'auger_on':
            c_msg.auger_on = value
            c_msg.enable[7] = True
        return c_msg

    async def receive_data(self, websocket, path):
        try:
            async for data in websocket:
                data = json.loads(data)
                c = data.get('control')
                s = data.get('schedule')
                z = data.get('zones')
                if c is not None:
                    msg = self.make_control(c)
                    # self.log(f'socket: {c}')
                    self.control_pub.publish(msg)
                elif s is not None:
                    s_msg = FlutterSchedule()
                    s_msg.time = int(s)  # epoch secs
                    # self.log('socket: schedule')
                    self.schedule_pub.publish(s_msg)
                elif z is not None:
                    zone_msg = FlutterZones()
                    zone_msg.clear_zone[0].lat = z['clear_zone'][0]['lat']
                    zone_msg.clear_zone[0].lon = z['clear_zone'][0]['lon']
                    zone_msg.clear_zone[1].lat = z['clear_zone'][1]['lat']
                    zone_msg.clear_zone[1].lon = z['clear_zone'][1]['lon']
                    zone_msg.snow_zone[0].lat = z['snow_zone'][0]['lat']
                    zone_msg.snow_zone[0].lon = z['snow_zone'][0]['lon']
                    zone_msg.snow_zone[1].lat = z['snow_zone'][1]['lat']
                    zone_msg.snow_zone[1].lon = z['snow_zone'][1]['lon']
                    # self.log('socket: zones')
                    self.zone_pub.publish(zone_msg)
                else:
                    self.log('FlutterNode:run - Nothing found in json')
        except Exception as e:
            self.log(f'Exception in receive socket: {e}')
    
    async def start_server(self):
        while True:
            try:
                async with websockets.serve(self.receive_data, "0.0.0.0", 8765):
                    await asyncio.Future()  # keep the server running
            except asyncio.exceptions.IncompleteReadError as e:
                self.log(f"IncompleteReadError, restarting: {e}")
            except Exception as e:
                self.log(f'Error in websocket, restarting: {e}')
            await asyncio.sleep(3)


def main():
    rclpy.init()
    node = FlutterNode()
    asyncio.run(node.start_server())
    rclpy.spin(node)

if __name__ == '__main__':
    main()