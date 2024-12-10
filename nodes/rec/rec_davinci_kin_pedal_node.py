#!/usr/bin/env python3

# ros libraries
from rclpy.node import Node
from rclpy.serialization import serialize_message
import rosbag2_py
import message_filters
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# custom libraries
from sitl_ros2_interfaces.msg import BoolStamped
from utils import ros2_utils

class REC_DVRK_KIN(Node):
    def __init__(self, params, topic_names):
        super().__init__(params['node_name'])

        # initialize bag
        self.bag_writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=params["save_path"] + "/kin_pedal",
            storage_id='sqlite3'
        )
        converter_options = rosbag2_py._storage.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        self.bag_writer.open(storage_options, converter_options)

        self.topics = topic_names
        self.create_bag_topics(topic_names)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            self.gen_subs(topic_names, params["queue_size"]),
            queue_size=params["queue_size"],
            slop=params["slop"]
        )
        self.ts.registerCallback(self.callback)

    def gen_subs(self, topic_names, qos_profile):
        subs = [] 
        for topic_name in topic_names:
            if 'cp' in topic_name:
                subs.append(
                    message_filters.Subscriber(self, PoseStamped, topic_name, qos_profile=qos_profile)
                )
            elif 'js' in topic_name:
                subs.append(
                    message_filters.Subscriber(self, JointState, topic_name, qos_profile=qos_profile)
                )
            elif 'pedal' in topic_name:
                subs.append(
                    message_filters.Subscriber(self, BoolStamped, topic_name, qos_profile=qos_profile)
                )
        return subs
    
    def create_bag_topics(self, topic_names):
        for topic_name in topic_names:
            if 'cp' in topic_name:
                topic_type = 'geometry_msgs/msg/PoseStamped'
            elif 'js' in topic_name:
                topic_type = 'sensor_msgs/msg/JointState'
            elif 'pedal' in topic_name:
                topic_type = 'sitl_ros2_interfaces/msg/BoolStamped'
            topic_metadata = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr'
            )
            self.bag_writer.create_topic(topic_metadata)

    def callback(self, *msgs):
        for i, msg in enumerate(msgs):
            self.bag_writer.write(
                self.topics[i],
                serialize_message(msg),
                self.get_clock().now().nanoseconds
            )
