#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import os
import numpy as np

from utils import tf_utils, ros2_utils, ik_utils

class PUB_CUSTOM_DVRK_CP(Node):
    def __init__(self, params):
        super().__init__(params["node_name"])
        self.load_tfs()
        self.psm1_params = ik_utils.get_arm_calib_data(params["psm1_calib_fn"])
        self.psm2_params = ik_utils.get_arm_calib_data(params["psm2_calib_fn"])
        self.ecm_params  = ik_utils.get_arm_calib_data(params["ecm_calib_fn"])
        self.g_ecm_dvrk  = tf_utils.g_ecm_dvrk(params["cam_type"])
        
        # Initialize publishers
        self.custom_psm1_cp           = self.create_publisher(PoseStamped, "/PSM1/custom/setpoint_cp", params["queue_size"])
        self.custom_psm2_cp           = self.create_publisher(PoseStamped, "/PSM2/custom/setpoint_cp", params["queue_size"])
        self.custom_psm1_jaw_cp       = self.create_publisher(PoseStamped, "/PSM1/custom/jaw/setpoint_cp", params["queue_size"])
        self.custom_psm2_jaw_cp       = self.create_publisher(PoseStamped, "/PSM2/custom/jaw/setpoint_cp", params["queue_size"])
        self.custom_ecm_cp            = self.create_publisher(PoseStamped, "/ECM/custom/setpoint_cp", params["queue_size"])
        self.custom_psm1_local_cp     = self.create_publisher(PoseStamped, "/PSM1/custom/local/setpoint_cp", params["queue_size"])
        self.custom_psm2_local_cp     = self.create_publisher(PoseStamped, "/PSM2/custom/local/setpoint_cp", params["queue_size"])
        self.custom_psm1_local_jaw_cp = self.create_publisher(PoseStamped, "/PSM1/custom/local/jaw/setpoint_cp", params["queue_size"])
        self.custom_psm2_local_jaw_cp = self.create_publisher(PoseStamped, "/PSM2/custom/local/jaw/setpoint_cp", params["queue_size"])
        self.custom_ecm_local_cp      = self.create_publisher(PoseStamped, "/ECM/custom/local/setpoint_cp", params["queue_size"])  

        # Initialize subscribers
        self.ecm_js  = self.create_subscription(JointState, "/ECM/setpoint_js",  self.ecm_callback,  params["queue_size"])
        self.psm1_js = self.create_subscription(JointState, "/PSM1/setpoint_js", self.psm1_callback, params["queue_size"])
        self.psm2_js = self.create_subscription(JointState, "/PSM2/setpoint_js", self.psm2_callback, params["queue_size"])
    
        # Initialize variables
        self.g_ecmtip_ecmbase = None

    def load_tfs(self):
        tf_path = "/home/" + os.getlogin() + "/aruco_data/base_tfs.yaml"
        tf_data = tf_utils.load_tf_data(tf_path)
        g_odom_psm1base = np.array(tf_data["g_odom_psm1base"])
        g_odom_psm2base = np.array(tf_data["g_odom_psm2base"])
        self.g_odom_ecmbase = np.array(tf_data["g_odom_ecmbase"])
        g_ecmbase_odom = tf_utils.ginv(self.g_odom_ecmbase)
        self.g_ecmbase_psm1base = g_ecmbase_odom.dot(g_odom_psm1base)
        self.g_ecmbase_psm2base = g_ecmbase_odom.dot(g_odom_psm2base)
        # Fix dataset with the correct transformation
        self.g_psm1tip_psm1jaw = tf_utils.g_psm1tip_psm1jaw
        self.g_psm2tip_psm2jaw = tf_utils.g_psm2tip_psm2jaw


    def ecm_callback(self, ecm_js):
        ecm_jp = np.array(ecm_js.position)
        g_ecmbase_ecmtip = ik_utils.get_tip_pose(ecm_jp, self.ecm_params).dot(self.g_ecm_dvrk)
        custom_ecm_local_cp_msg = tf_utils.g2posestamped(g_ecmbase_ecmtip, ros2_utils.now(self), "ECM_base")
        self.custom_ecm_local_cp.publish(custom_ecm_local_cp_msg)

        g_odom_ecmtip = self.g_odom_ecmbase.dot(g_ecmbase_ecmtip)
        custom_ecm_cp_msg = tf_utils.g2posestamped(g_odom_ecmtip, ros2_utils.now(self), "Cart")
        self.custom_ecm_cp.publish(custom_ecm_cp_msg)

        self.g_ecmtip_ecmbase = tf_utils.ginv(g_ecmbase_ecmtip)

    def psm1_callback(self, psm1_js):
        psm1_jp = np.array(psm1_js.position)

        g_psm1base_psm1tip = ik_utils.get_tip_pose(psm1_jp, self.psm1_params)
        custom_psm1_local_cp_msg = tf_utils.g2posestamped(g_psm1base_psm1tip, ros2_utils.now(self), "PSM1_base")
        self.custom_psm1_local_cp.publish(custom_psm1_local_cp_msg)

        g_psm1base_psm1jaw = g_psm1base_psm1tip.dot(self.g_psm1tip_psm1jaw)
        custom_psm1_local_jaw_cp_msg = tf_utils.g2posestamped(g_psm1base_psm1jaw, ros2_utils.now(self), "PSM1_base")
        self.custom_psm1_local_jaw_cp.publish(custom_psm1_local_jaw_cp_msg)

        if self.g_ecmtip_ecmbase is None:
            return

        g_ecmtip_psm1tip = self.g_ecmtip_ecmbase.dot(self.g_ecmbase_psm1base).dot(g_psm1base_psm1tip)
        custom_psm1_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm1tip, ros2_utils.now(self), "ECM")
        self.custom_psm1_cp.publish(custom_psm1_cp_msg)

        g_ecmtip_psm1jaw = g_ecmtip_psm1tip.dot(self.g_psm1tip_psm1jaw)
        custom_psm1_jaw_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm1jaw, ros2_utils.now(self), "ECM")
        self.custom_psm1_jaw_cp.publish(custom_psm1_jaw_cp_msg)

    def psm2_callback(self, psm2_js):
        psm2_jp = np.array(psm2_js.position)

        g_psm2base_psm2tip = ik_utils.get_tip_pose(psm2_jp, self.psm2_params)
        custom_psm2_local_cp_msg = tf_utils.g2posestamped(g_psm2base_psm2tip, ros2_utils.now(self), "PSM2_base")
        self.custom_psm2_local_cp.publish(custom_psm2_local_cp_msg)

        g_psm2base_psm2jaw = g_psm2base_psm2tip.dot(self.g_psm2tip_psm2jaw)
        custom_psm2_local_jaw_cp_msg = tf_utils.g2posestamped(g_psm2base_psm2jaw, ros2_utils.now(self), "PSM2_base")
        self.custom_psm2_local_jaw_cp.publish(custom_psm2_local_jaw_cp_msg)

        if self.g_ecmtip_ecmbase is None:
            return

        g_ecmtip_psm2tip = self.g_ecmtip_ecmbase.dot(self.g_ecmbase_psm2base).dot(g_psm2base_psm2tip)
        custom_psm2_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm2tip, ros2_utils.now(self), "ECM")
        self.custom_psm2_cp.publish(custom_psm2_cp_msg)

        g_ecmtip_psm2jaw = g_ecmtip_psm2tip.dot(self.g_psm2tip_psm2jaw)
        custom_psm2_jaw_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm2jaw, ros2_utils.now(self), "ECM")
        self.custom_psm2_jaw_cp.publish(custom_psm2_jaw_cp_msg)
