from rclpy.node import Node

from pynput import keyboard

from sitl_ros2_interfaces.msg import StringStamped
from utils import ros2_utils

class DISSECT_KEY(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])
        qos_profile = ros2_utils.custom_qos_profile(params['queue_size'])
        self.msg = StringStamped()
        self.pub_key = self.create_publisher(StringStamped, "keyboard/dissect", qos_profile)
        self.char_list = ['a', 'f', 'r', 'i', 'b']
        ros2_utils.loginfo(self, "Press one of the following keys...")
        ros2_utils.loginfo(self, "a: align instrument before following boundary.")
        ros2_utils.loginfo(self, "f: follow the tissue boundary")
        ros2_utils.loginfo(self, "r: return to the position before following boundary.")
        ros2_utils.loginfo(self, "i: go to the initial position.")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        if hasattr(key, 'char') and key.char in self.char_list:
            self.msg.header.stamp = ros2_utils.now(self)
            self.msg.data = str(key.char)
            self.pub_key.publish(self.msg)
        else:
            ros2_utils.loginfo(self, f"Special key '{key}' pressed")
