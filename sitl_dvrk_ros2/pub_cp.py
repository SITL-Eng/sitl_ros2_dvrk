#!/usr/bin/env python3

import os
import numpy as np
import math
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber

from sitl_ros2_dvrk.utils import tf_utils, aruco_utils, ik_devel_utils



class PUB_CUSTOM_DVRK_CP(Node):
    def __init__(self, params):
        super().__init__('pub_cp')
        self.params = params
        self.load_tfs()
        self.load_params()
        if self.params["cam_type"] == "30":
            self.g_ecm_dvrk  = tf_utils.cv2vecs2g(np.array([1,0,0])*math.radians(30),np.array([0,0,0]))
        elif self.params["cam_type"] == "0":
            self.g_ecm_dvrk  = np.eye(4)
        
        # Initialize publishers
        self.custom_psm1_cp = self.create_publisher(PoseStamped, "/PSM1/custom/setpoint_cp", 10)
        self.custom_psm2_cp = self.create_publisher(PoseStamped, "/PSM2/custom/setpoint_cp", 10)
        self.custom_psm1_jaw_cp = self.create_publisher(PoseStamped, "/PSM1/custom/jaw/setpoint_cp", 10)
        self.custom_psm2_jaw_cp = self.create_publisher(PoseStamped, "/PSM2/custom/jaw/setpoint_cp", 10)
        self.custom_ecm_cp = self.create_publisher(PoseStamped, "/ECM/custom/setpoint_cp", 10)
        self.custom_psm1_local_cp = self.create_publisher(PoseStamped, "/PSM1/custom/local/setpoint_cp", 10)
        self.custom_psm2_local_cp = self.create_publisher(PoseStamped, "/PSM2/custom/local/setpoint_cp", 10)
        self.custom_psm1_local_jaw_cp = self.create_publisher(PoseStamped, "/PSM1/custom/local/jaw/setpoint_cp", 10)
        self.custom_psm2_local_jaw_cp = self.create_publisher(PoseStamped, "/PSM2/custom/local/jaw/setpoint_cp", 10)
        self.custom_ecm_local_cp = self.create_publisher(PoseStamped, "/ECM/custom/local/setpoint_cp", 10)

        

        # Initialize subscribers
        self.psm1_js = Subscriber(self, JointState, "/PSM1/setpoint_js")
        self.psm2_js = Subscriber(self, JointState, "/PSM2/setpoint_js")
        self.ecm_js = Subscriber(self, JointState, "/ECM/setpoint_js")

        # Set up time synchronizer
        ts = ApproximateTimeSynchronizer([self.psm1_js, self.psm2_js, self.ecm_js], 10, 0.01)
        ts.registerCallback(self.callback)

    def load_tfs(self):
        tf_path = "/home/" + os.getlogin() + "/aruco_data/base_tfs.yaml"
        tf_data = aruco_utils.load_tf_data(tf_path)
        g_odom_psm1base = np.array(tf_data["g_odom_psm1base"])
        g_odom_psm2base = np.array(tf_data["g_odom_psm2base"])
        self.g_odom_ecmbase = np.array(tf_data["g_odom_ecmbase"])
        g_ecmbase_odom = tf_utils.ginv(self.g_odom_ecmbase)
        self.g_ecmbase_psm1base = g_ecmbase_odom.dot(g_odom_psm1base)
        self.g_ecmbase_psm2base = g_ecmbase_odom.dot(g_odom_psm2base)
        # Fix dataset with the correct transformation
        # self.g_psm1tip_psm1jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([0.002,-0.0035,0.015]))
        # self.g_psm2tip_psm2jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([0.002,-0.0035,0.015]))
        self.g_psm1tip_psm1jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([-0.005,-0.0025,0.0147]))
        self.g_psm2tip_psm2jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([-0.0039, 0.0, 0.02]))
    
    def load_params(self):
        self.psm1_params = ik_devel_utils.get_arm_calib_data(self.params["psm1_calib_fn"])
        self.psm2_params = ik_devel_utils.get_arm_calib_data(self.params["psm2_calib_fn"])
        self.ecm_params  = ik_devel_utils.get_arm_calib_data(self.params["ecm_calib_fn"])

    def callback(self, psm1_js, psm2_js, ecm_js):
        psm1_jp = np.array(psm1_js.position)
        g_psm1base_psm1tip = ik_devel_utils.get_tip_pose(psm1_jp,self.psm1_params)
        g_psm1base_psm1jaw = g_psm1base_psm1tip.dot(self.g_psm1tip_psm1jaw)
        psm2_jp = np.array(psm2_js.position)
        g_psm2base_psm2tip = ik_devel_utils.get_tip_pose(psm2_jp,self.psm2_params)
        g_psm2base_psm2jaw = g_psm2base_psm2tip.dot(self.g_psm2tip_psm2jaw)
        ecm_jp  = np.array(ecm_js.position)
        g_ecmbase_ecmtip = ik_devel_utils.get_tip_pose(ecm_jp,self.ecm_params).dot(self.g_ecm_dvrk)
        g_ecmtip_ecmbase = tf_utils.ginv(g_ecmbase_ecmtip)
        g_ecmtip_psm1tip = g_ecmtip_ecmbase.dot(self.g_ecmbase_psm1base).dot(g_psm1base_psm1tip)
        g_ecmtip_psm1jaw = g_ecmtip_psm1tip.dot(self.g_psm1tip_psm1jaw)
        g_ecmtip_psm2tip = g_ecmtip_ecmbase.dot(self.g_ecmbase_psm2base).dot(g_psm2base_psm2tip)
        g_ecmtip_psm2jaw = g_ecmtip_psm2tip.dot(self.g_psm2tip_psm2jaw)
        g_odom_ecmtip = self.g_odom_ecmbase.dot(g_ecmbase_ecmtip)

        t = self.get_clock().now().to_msg()

        # rvec = cv2.Rodrigues(g_odom_ecmtip[:3,:3])[0]
        # quat = tf_utils.rvec2quat(rvec)
        # self.get_logger().info(f"{quat[0]}")

        custom_ecm_cp_msg = tf_utils.g2posestamped(g_odom_ecmtip,t,"Cart")
        self.custom_ecm_cp.publish(custom_ecm_cp_msg)

        custom_psm1_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm1tip,t,"ECM")
        self.custom_psm1_cp.publish(custom_psm1_cp_msg)

        custom_psm2_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm2tip,t,"ECM")
        self.custom_psm2_cp.publish(custom_psm2_cp_msg)

        custom_psm1_jaw_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm1jaw,t,"ECM")
        self.custom_psm1_jaw_cp.publish(custom_psm1_jaw_cp_msg)

        custom_psm2_jaw_cp_msg = tf_utils.g2posestamped(g_ecmtip_psm2jaw,t,"ECM")
        self.custom_psm2_jaw_cp.publish(custom_psm2_jaw_cp_msg)

        custom_psm1_local_cp_msg = tf_utils.g2posestamped(g_psm1base_psm1tip,t,"PSM1_base")
        self.custom_psm1_local_cp.publish(custom_psm1_local_cp_msg)

        custom_psm2_local_cp_msg = tf_utils.g2posestamped(g_psm2base_psm2tip,t,"PSM2_base")
        self.custom_psm2_local_cp.publish(custom_psm2_local_cp_msg)

        custom_ecm_local_cp_msg = tf_utils.g2posestamped(g_ecmbase_ecmtip,t,"ECM_base")
        self.custom_ecm_local_cp.publish(custom_ecm_local_cp_msg)

        custom_psm1_local_jaw_cp_msg = tf_utils.g2posestamped(g_psm1base_psm1jaw,t,"PSM1_base")
        self.custom_psm1_local_jaw_cp.publish(custom_psm1_local_jaw_cp_msg)

        custom_psm2_local_jaw_cp_msg = tf_utils.g2posestamped(g_psm2base_psm2jaw,t,"PSM2_base")
        self.custom_psm1_local_jaw_cp.publish(custom_psm2_local_jaw_cp_msg)

        
def main(args=None):
    rclpy.init(args=args)
    params = {
        "cam_type" : "30",
        "psm1_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm1_calib_results_final_new_v2.mat",
        "psm2_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm2_calib_results_final_new_v2.mat",
        "ecm_calib_fn"  : "/home/" + os.getlogin() + "/aruco_data/ecm_calib_results_final_new_v2.mat"
    }
    node = PUB_CUSTOM_DVRK_CP(params)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
