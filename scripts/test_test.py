#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
from rtcrobot_interfaces.msg import Nav350Data, Nav350Reflector, Reflector, Nav350Mode, AddLandmark, EditLandmark
import math
import struct
# from rtcrobot_interfaces import enum_common as enum
# kien: because of transformations
# import tf2
import tf2_ros
import psycopg2
# import tf2_conversions
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
from rtcrobot_interfaces.srv import SwitchMap
# import rtcrobot_interfaces.mapdata as mapdata
import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '/home/nth/chin_ws/src/rtcrobot_interfaces/rtcrobot_interfaces/')
import log
import enum_common as enum
import time
import traceback
from rclpy.executors import MultiThreadedExecutor
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SwitchMap, '/switch_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SwitchMap.Request()

    def send_request(self, layer, x,y,phi):
        self.req.map_layer = layer
        self.req.x = x
        self.req.y = y
        self.req.phi = phi
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 0.0, 0.0, 0.0)
    
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
