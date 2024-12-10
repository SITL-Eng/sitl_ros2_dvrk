#!/usr/bin/env python3

# ROS2 Libraries
from rclpy.node import Node
from utils import ros2_utils
from sitl_ros2_interfaces.msg import BoolStamped

# Python Libraries
import serial
import time

class PUB_PEDAL_MP_R(Node):
    def __init__(self, params):
        # Setup ROS2 Publisher
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.pub_pedal_mp_r = self.create_publisher(BoolStamped, '/pedal/monopolar/read', qos_profile)
        self.timer = self.create_timer(1/params["hz"], self.pedal_mp_r_cb)

        # Setup arduino connection
        self.arduino = serial.Serial(port=params["port"], baudrate=params["baud"], timeout=1)
        time.sleep(2) # Wait for serial connection to establish

    def pedal_mp_r_cb(self):
        self.arduino.write('R'.encode())
        sensor_val = self.arduino.readline().decode().strip()
        msg = BoolStamped()
        msg.header.stamp = ros2_utils.now(self)
        if sensor_val == '1':
            msg.data = True
        elif sensor_val == '0':
            msg.data = False
        self.pub_pedal_mp_r.publish(msg)
