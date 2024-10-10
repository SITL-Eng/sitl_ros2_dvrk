import numpy as np
import math
import os
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from sitl_dvrk_ros2.utils import tf_utils, aruco_utils, ik_utils

class PUB_CUSTOM_DVRK_TF(Node):
    def __init__(self, params):
        super().__init__('pub_tf')
        self.params = params
        self.load_tfs()
        self.load_params()
        self.br = TransformBroadcaster(self)

        # Create subscriptions
        self.psm1_js = self.create_subscription(JointState, '/PSM1/setpoint_js', self.psm1_callback, 10)
        self.psm2_js = self.create_subscription(JointState, '/PSM2/setpoint_js', self.psm2_callback, 10)
        self.ecm_js = self.create_subscription(JointState, '/ECM/setpoint_js', self.ecm_callback, 10)

    def __del__(self):
        self.get_logger().info("Shutting down...")


    def load_tfs(self):
        tf_path = "/home/" + os.getlogin() + "/aruco_data/base_tfs.yaml"
        tf_data = aruco_utils.load_tf_data(tf_path)
        g_odom_psm1base = np.array(tf_data["g_odom_psm1base"])
        g_odom_psm2base = np.array(tf_data["g_odom_psm2base"])
        g_odom_ecmbase  = np.array(tf_data["g_odom_ecmbase"])
        g_odom_zed      = np.array(tf_data["g_odom_zed"])
        g_map_odom      = tf_utils.cv2vecs2g(np.array([0,0,1])*math.radians(90),np.array([0,0,1])).dot(
            tf_utils.cv2vecs2g(np.array([1,0,0])*math.radians(90),np.array([0,0,0])))
        g_psm1tip_psm1jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([-0.005,-0.0025,0.0147]))
        g_psm2tip_psm2jaw = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([-0.004, 0.0, 0.019]))
        
        # cam_type = self.get_parameter('cam_type').value
        if self.params['cam_type'] == "30":
            self.g_ecm_dvrk  = tf_utils.cv2vecs2g(np.array([1,0,0])*math.radians(30),np.array([0,0,0]))
        elif self.params['cam_type'] == "0":
            self.g_ecm_dvrk  = tf_utils.cv2vecs2g(np.array([0.0,0.0,0.0]),np.array([0,0,0]))
        
        self.g_ecmdvrk_ecmopencv = np.array(tf_data["g_ecmdvrk_ecmopencv"])
        self.zed_test_tf         = tf_utils.g2tf(np.array(tf_data["g_zed_test"]))
        self.odom_psm1base_tf    = tf_utils.g2tf(g_odom_psm1base)
        self.odom_psm2base_tf    = tf_utils.g2tf(g_odom_psm2base)
        self.odom_ecmbase_tf     = tf_utils.g2tf(g_odom_ecmbase)
        self.odom_zed_tf         = tf_utils.g2tf(g_odom_zed)
        self.map_odom_tf         = tf_utils.g2tf(g_map_odom)
        self.psm1tip_psm1jaw_tf  = tf_utils.g2tf(g_psm1tip_psm1jaw)
        self.psm2tip_psm2jaw_tf  = tf_utils.g2tf(g_psm2tip_psm2jaw)

    def load_params(self):
        self.psm1_params = ik_utils.get_arm_calib_data(self.params['psm1_calib_fn'])
        self.psm2_params = ik_utils.get_arm_calib_data(self.params['psm2_calib_fn'])
        self.ecm_params  = ik_utils.get_arm_calib_data(self.params['ecm_calib_fn'])

    def psm1_callback(self, psm1_js):
        t = self.get_clock().now().to_msg()
        self.broadcast_transform(t, 'psm1_base', 'odom', self.odom_psm1base_tf)
        psm1_jp = np.array(psm1_js.position)
        g_psm1base_psm1tip = ik_utils.get_tip_pose(psm1_jp, self.psm1_params)
        psm1base_psm1tip_tf = tf_utils.g2tf(g_psm1base_psm1tip)
        self.broadcast_transform(t, 'psm1_tip', 'psm1_base', psm1base_psm1tip_tf)
        self.broadcast_transform(t, 'psm1_jaw', 'psm1_tip', self.psm1tip_psm1jaw_tf)

    def psm2_callback(self, psm2_js):
        t = self.get_clock().now().to_msg()
        self.broadcast_transform(t, 'psm2_base', 'odom', self.odom_psm2base_tf)
        psm2_jp = np.array(psm2_js.position)
        g_psm2base_psm2tip = ik_utils.get_tip_pose(psm2_jp, self.psm2_params)
        psm2base_psm2tip_tf = tf_utils.g2tf(g_psm2base_psm2tip)
        self.broadcast_transform(t, 'psm2_tip', 'psm2_base', psm2base_psm2tip_tf)
        self.broadcast_transform(t, 'psm2_jaw', 'psm2_tip', self.psm2tip_psm2jaw_tf)

    def ecm_callback(self, ecm_js):
        t = self.get_clock().now().to_msg()
        self.broadcast_transform(t, 'odom', 'map', self.map_odom_tf)
        self.broadcast_transform(t, 'base_link', 'odom', self.odom_zed_tf)
        self.broadcast_transform(t, 'ecm_base', 'odom', self.odom_ecmbase_tf)
        self.broadcast_transform(t, 'test', 'base_link', self.zed_test_tf)
        
        ecm_jp = np.array(ecm_js.position)
        g_ecmbase_ecmdvrk = ik_utils.get_tip_pose(ecm_jp, self.ecm_params).dot(self.g_ecm_dvrk)
        ecmbase_ecmdvrk_tf = tf_utils.g2tf(g_ecmbase_ecmdvrk)
        ecmdvrk_ecmopencv_tf = tf_utils.g2tf(self.g_ecmdvrk_ecmopencv)
        self.broadcast_transform(t, 'ecm_tip', 'ecm_base', ecmbase_ecmdvrk_tf)
        self.broadcast_transform(t, 'ecm_left', 'ecm_tip', ecmdvrk_ecmopencv_tf)

    def broadcast_transform(self, timestamp, child_frame, parent_frame, transform):
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = transform.translation.x
        t.transform.translation.y = transform.translation.y
        t.transform.translation.z = transform.translation.z
        t.transform.rotation.x = transform.rotation.x
        t.transform.rotation.y = transform.rotation.y
        t.transform.rotation.z = transform.rotation.z
        t.transform.rotation.w = transform.rotation.w
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    params = {
        "cam_type" : "30",
        "psm1_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm1_calib_results_final_new_v2.mat",
        "psm2_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm2_calib_results_final_new_v2.mat",
        "ecm_calib_fn"  : "/home/" + os.getlogin() + "/aruco_data/ecm_calib_results_final_new_v2.mat"
    }

    node = PUB_CUSTOM_DVRK_TF(params)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
