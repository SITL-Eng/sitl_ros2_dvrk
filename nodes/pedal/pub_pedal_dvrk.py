#! /usr/bin/env python3
from rclpy.node import Node
from sensor_msgs.msg import Joy

from utils import ros2_utils
from sitl_ros2_interfaces.msg import BoolStamped

class PUB_DVRK_PEDALS(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        self.camera_msg = BoolStamped()
        self.camera_msg.data = False
        self.clutch_msg = BoolStamped()
        self.clutch_msg.data = False
        self.sub_clutch = self.create_subscription(Joy, "/footpedals/clutch", self.clutch_cb, params["queue_size"])
        self.sub_camera = self.create_subscription(Joy, "/footpedals/camera", self.camera_cb, params["queue_size"])
        self.pub_clutch = self.create_publisher(BoolStamped, "/pedal/clutch", params["queue_size"])
        self.pub_camera = self.create_publisher(BoolStamped, "/pedal/camera", params["queue_size"])
        self.loop_rate = self.create_timer(1/params["pub_rate"], self.callback)
        
    def clutch_cb(self, clutch_joy):
        if clutch_joy.buttons[0] == 0:
            self.clutch_msg.data = False
        elif clutch_joy.buttons[0] == 1:
            self.clutch_msg.data = True

    def camera_cb(self, camera_joy):
        if camera_joy.buttons[0] == 0:
            self.camera_msg.data = False
        elif camera_joy.buttons[0] == 1:
            self.camera_msg.data = True

    def callback(self):
        self.clutch_msg.header.stamp = ros2_utils.now(self)
        self.pub_clutch.publish(self.clutch_msg)
        self.camera_msg.header.stamp = ros2_utils.now(self)
        self.pub_camera.publish(self.camera_msg)