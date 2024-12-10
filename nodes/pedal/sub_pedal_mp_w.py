#!/usr/bin/env python3

# ROS2 Libraries
from rclpy.node import Node
from utils import ros2_utils
from sitl_ros2_interfaces.msg import BoolStamped

# Python Libraries
import serial
import time

class SUB_PEDAL_MP_W(Node):
    def __init__(self, params):
        # Setup ROS2 Subscriber
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.sub_pedal_mp_w  = self.create_subscription(
            BoolStamped,
            '/pedal/monopolar/write',
            self.pedal_mp_w_cb,
            qos_profile
        )

        # Setup arduino connection
        self.arduino = serial.Serial(port=params["port"], baudrate=params["baud"], timeout=1)
        time.sleep(2) # Wait for serial connection to establish

    def pedal_mp_w_cb(self, msg):
        command = msg.data
        if command:
            self.arduino.write('H'.encode())
        else:
            self.arduino.write('L'.encode())
