from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

import numpy as np
import os

from utils import tf_utils, ros2_utils, ik_utils

class PUB_CUSTOM_DVRK_TF(Node):
    def __init__(self, params):
        super().__init__(params['node_name'])

        self.br = TransformBroadcaster(self)
        self.init_tfs(params['cam_type'])
        self.psm1_params = ik_utils.get_arm_calib_data(params['psm1_calib_fn'])
        self.psm2_params = ik_utils.get_arm_calib_data(params['psm2_calib_fn'])
        self.ecm_params  = ik_utils.get_arm_calib_data(params['ecm_calib_fn'])

        # Create subscriptions
        self.psm1_js = self.create_subscription(
            JointState,
            '/PSM1/setpoint_js',
            self.psm1_callback,
            params['queue_size']
        )
        self.psm2_js = self.create_subscription(
            JointState,
            '/PSM2/setpoint_js',
            self.psm2_callback,
            params['queue_size']
        )
        self.ecm_js = self.create_subscription(
            JointState,
            '/ECM/setpoint_js',
            self.ecm_callback,
            params['queue_size']
        )

    def init_tfs(self, cam_type):
        tf_path = "/home/" + os.getlogin() + "/aruco_data/base_tfs.yaml"
        tf_data = tf_utils.load_tf_data(tf_path)
        g_odom_psm1base   = np.array(tf_data["g_odom_psm1base"])
        g_odom_psm2base   = np.array(tf_data["g_odom_psm2base"])
        g_odom_ecmbase    = np.array(tf_data["g_odom_ecmbase"])
        g_odom_zed        = np.array(tf_data["g_odom_zed"])
        g_map_odom        = tf_utils.g_map_odom
        g_psm1tip_psm1jaw = tf_utils.g_psm1tip_psm1jaw
        g_psm2tip_psm2jaw = tf_utils.g_psm2tip_psm2jaw
        self.g_ecm_dvrk   = tf_utils.g_ecm_dvrk(cam_type)
        g_ecmdvrk_ecmopencv = np.array(tf_data["g_ecmdvrk_ecmopencv"])

        self.odom_zed_tf = tf_utils.g2tfstamped(
            g_odom_zed,
            ros2_utils.now(self),
            'odom',
            'base_link'
        )
        self.odom_psm1base_tf = tf_utils.g2tfstamped(
            g_odom_psm1base,
            ros2_utils.now(self),
            'odom',
            'psm1_base'
        )
        self.odom_psm2base_tf = tf_utils.g2tfstamped(
            g_odom_psm2base,
            ros2_utils.now(self),
            'odom',
            'psm2_base'
        )
        self.odom_ecmbase_tf = tf_utils.g2tfstamped(
            g_odom_ecmbase,
            ros2_utils.now(self),
            'odom',
            'ecm_base'
        )
        self.map_odom_tf = tf_utils.g2tfstamped(
            g_map_odom,
            ros2_utils.now(self),
            'map',
            'odom'
        )
        self.psm1tip_psm1jaw_tf = tf_utils.g2tfstamped(
            g_psm1tip_psm1jaw,
            ros2_utils.now(self),
            'psm1_tip',
            'psm1_jaw'
        )
        self.psm2tip_psm2jaw_tf = tf_utils.g2tfstamped(
            g_psm2tip_psm2jaw,
            ros2_utils.now(self),
            'psm2_tip',
            'psm2_jaw'
        )
        self.ecmdvrk_ecmopencv_tf = tf_utils.g2tfstamped(
            g_ecmdvrk_ecmopencv,
            ros2_utils.now(self),
            'ecm_tip',
            'ecm_left'
        )

    def psm1_callback(self, psm1_js):
        psm1base_psm1tip_tf = tf_utils.g2tfstamped(
            ik_utils.get_tip_pose(
                np.array(psm1_js.position),
                self.psm1_params
            ),
            ros2_utils.now(self),
            'psm1_base',
            'psm1_tip'
        )
        self.odom_psm1base_tf.header.stamp = self.psm1tip_psm1jaw_tf.header.stamp = psm1base_psm1tip_tf.header.stamp
        self.br.sendTransform([self.odom_psm1base_tf, psm1base_psm1tip_tf, self.psm1tip_psm1jaw_tf])

    def psm2_callback(self, psm2_js):
        psm2base_psm2tip_tf = tf_utils.g2tfstamped(
            ik_utils.get_tip_pose(
                np.array(psm2_js.position),
                self.psm2_params
            ),
            ros2_utils.now(self),
            'psm2_base',
            'psm2_tip'
        )
        self.odom_psm2base_tf.header.stamp = self.psm2tip_psm2jaw_tf.header.stamp = psm2base_psm2tip_tf.header.stamp
        self.br.sendTransform([self.odom_psm2base_tf, psm2base_psm2tip_tf, self.psm1tip_psm1jaw_tf])

    def ecm_callback(self, ecm_js):
        ecmbase_ecmdvrk_tf = tf_utils.g2tfstamped(
            ik_utils.get_tip_pose(
                np.array(ecm_js.position),
                self.ecm_params
            ).dot(self.g_ecm_dvrk),
            ros2_utils.now(self),
            'ecm_base',
            'ecm_tip'
        )
        self.map_odom_tf.header.stamp = self.odom_ecmbase_tf.header.stamp = self.ecmdvrk_ecmopencv_tf.header.stamp = ecmbase_ecmdvrk_tf.header.stamp
        self.br.sendTransform([self.map_odom_tf, self.odom_ecmbase_tf, ecmbase_ecmdvrk_tf, self.ecmdvrk_ecmopencv_tf])
