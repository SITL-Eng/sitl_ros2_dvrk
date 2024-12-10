#!/usr/bin/env python3

# ROS2 Libraries
from rclpy.node import Node
from utils import ros2_utils
from sitl_ros2_interfaces.msg import BoolStamped

class PUB_PEDAL_MP_W(Node):
    def __init__(self, params):
        # Setup ROS2 Publisher
        super().__init__(params["node_name"])
        qos_profile = ros2_utils.custom_qos_profile(params["queue_size"])
        self.pub_pedal_mp_r = self.create_publisher(BoolStamped, '/pedal/monopolar/write', qos_profile)
        self.timer = self.create_timer(1/params["hz"], self.pedal_mp_w_cb)

        if params["command"] == 'H':
            self.command = True
        elif params["command"] == 'L':
            self.command = False

    def pedal_mp_w_cb(self):
        msg = BoolStamped()
        msg.header.stamp = ros2_utils.now(self)

        # Toggle between high and low (True and False)
        self.command = not self.command
        
        # Set the message data based on the current state
        msg.data = self.command
        self.pub_pedal_mp_r.publish(msg)
