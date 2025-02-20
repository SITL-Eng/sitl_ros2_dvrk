# ROS2 Libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped

# Python Libraries
import math
import numpy as np
from scipy.optimize import minimize

# dVRK
import dvrk

# Custom Libraries 
from sitl_ros2_interfaces.msg import StringStamped
from utils import tf_utils, pcl_utils, misc_utils, ik_utils, dvrk_utils

class GRASP:
    def __init__(self, ral, params):
        self.params        = params
        self.ral           = ral
        self.key           = None
        self.ctrd_3d       = None
        self.grasp_3d      = None
        self.skel_3d       = None
        self.bnd_3d        = None
        self.g_fbf         = None
        self.g_fbfjaw      = None
        self.grasp_dir     = None
        self.jp_align      = None
        self.jp_grasp      = None
        self.jp_pull       = None
        self.jp_release    = None
        self.tissue_norm   = None
        self.custom_arm_cp = None
        self.custom_local_arm_cp = None
        self.custom_jaw_cp = None
        self.custom_local_jaw_cp = None
        self.expected_interval = params['expected_interval']
        self.arm_cp_tn = "/{}/custom/setpoint_cp".format(params['arm_name'])
        self.arm_local_cp_tn = "/{}/custom/local/setpoint_cp".format(params['arm_name'])
        if "PSM" in params['arm_name']:
            self.jaw_cp_tn = "/{}/custom/jaw/setpoint_cp".format(params['arm_name'])
            self.jaw_local_cp_tn = "/{}/custom/local/jaw/setpoint_cp".format(params['arm_name'])
            self.arm = dvrk.psm(
                ral = ral,
                arm_name = params['arm_name'],
                expected_interval = self.expected_interval
            )
        elif "MTM" in params['arm_name']:
            self.arm = dvrk.mtm(
                ral = ral,
                arm_name = params['arm_name'],
                expected_interval = self.expected_interval
            )
            self.arm.jaw = self.arm.gripper
        else:
            self.arm = dvrk.arm(
                ral = ral,
                arm_name = params['arm_name'],
                expected_interval = self.expected_interval
            )
        self.sleep_rate = self.ral.create_rate(1.0 / self.expected_interval)
        self.g_ecmopencv_ecmdvrk = self.load_tf(params["tf_path"])
        self.align_dist     = params["align_dist"]
        self.grasp_offset   = params["grasp_offset"]
        self.align_ratio    = params["align_ratio"]
        # self.min_pull_dist  = params["min_pull_dist"]
        self.jaw_open_angle = params["jaw_open_angle"]
        self.max_pull_dist  = params["max_pull_dist"]

    def add_arm_ik(self):
        self.init_jp = dvrk_utils.get_jp(self.arm)
        self.ral.loginfo(f"init_jp: {self.init_jp}")
        self.align_ik = ik_utils.dvrk_custom_ik(
            self.params["calib_fn"],
            self.params["alignW"][0],
            self.params["alignW"][1],
            self.init_jp,
            self.params["Joffsets"]
        )
        self.move_ik = ik_utils.dvrk_custom_ik(
            self.params["calib_fn"],
            self.params["moveW"][0],
            self.params["moveW"][1],
            self.init_jp,
            self.params["Joffsets"]
        )

    def custom_arm_cp_callback(self, msg):
        self.custom_arm_cp = tf_utils.posestamped2g(msg)

    def custom_local_arm_cp_callback(self, msg):
        self.custom_local_arm_cp = tf_utils.posestamped2g(msg)

    def custom_jaw_cp_callback(self, msg):
        self.custom_jaw_cp = tf_utils.posestamped2g(msg)

    def custom_local_jaw_cp_callback(self, msg):
        self.custom_local_jaw_cp = tf_utils.posestamped2g(msg)

    def key_cb(self, key_msg):
        self.key = key_msg.data

    def bnd_3d_cb(self, bnd_3d_msg):
        self.bnd_3d = pcl_utils.pcl2nparray(bnd_3d_msg)

    def skel_3d_cb(self, skel_3d_msg):
        self.skel_3d = pcl_utils.pcl2nparray(skel_3d_msg)

    def ctrd_3d_cb(self, ctrd_3d_msg):
        self.ctrd_3d = tf_utils.ptstamped2pt3d(ctrd_3d_msg)

    def fbf_kpt_cb(self, fbf_cp_msg):
        self.g_fbf = tf_utils.tfstamped2g(fbf_cp_msg)

    def fbf_jaw_kpt_cb(self, fbf_jaw_cp_msg):
        self.g_fbfjaw = tf_utils.tfstamped2g(fbf_jaw_cp_msg)
    
    def add_subs(self):
        # Create a subscriber for the arm's cartesian position
        self.custom_arm_cp_sub = self.ral.subscriber(
            self.arm_cp_tn,
            PoseStamped,
            self.custom_arm_cp_callback,
        )
        # Create a subscriber for the arm's local cartesian position
        self.custom_arm_local_cp_sub = self.ral.subscriber(
            self.arm_local_cp_tn,
            PoseStamped,
            self.custom_local_arm_cp_callback,
        )
        # Create a subscriber for the jaw's cartesian position (if applicable)
        if hasattr(self, 'jaw_cp_tn'):
            self.custom_jaw_cp_sub = self.ral.subscriber(
                self.jaw_cp_tn,
                PoseStamped,
                self.custom_jaw_cp_callback,
            )
        # Create a subscriber for the jaw's local cartesian position (if applicable)
        if hasattr(self, 'jaw_local_cp_tn'):
            self.custom_jaw_local_cp_sub = self.ral.subscriber(
                self.jaw_local_cp_tn,
                PoseStamped,
                self.custom_local_jaw_cp_callback,
            )
        self.key_sub = self.ral.subscriber(
            self.params['key_topic'],
            StringStamped,
            self.key_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['skel_topic'],
            PointCloud2,
            self.skel_3d_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['bnd_topic'],
            PointCloud2,
            self.bnd_3d_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['ctrd_topic'],
            PointStamped,
            self.ctrd_3d_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['fbf_kpt_topic'],
            TransformStamped,
            self.fbf_kpt_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['fbf_jaw_kpt_topic'],
            TransformStamped,
            self.fbf_jaw_kpt_cb,
            self.params['queue_size']
        )
    
    def reset(self):
        self.grasp_dir   = None
        self.jp_align    = None
        self.jp_grasp    = None
        self.jp_pull     = None
        self.jp_release  = None
        self.tissue_norm = None

    def load_tf(self, tf_path):
        tf_data = tf_utils.load_tf_data(tf_path)
        g_ecmdvrk_ecmopencv = np.array(tf_data["g_ecmdvrk_ecmopencv"])
        return tf_utils.ginv(g_ecmdvrk_ecmopencv)
    
    def get_tf(self, dir_vec, des_loc):
        z_tf = dir_vec
        y_tf = misc_utils.unit_vector(np.cross(dir_vec, np.array([0, 0, -1])))
        x_tf = misc_utils.unit_vector(np.cross(y_tf, dir_vec))
        R_tf = np.vstack((x_tf, y_tf, z_tf)).T
        return tf_utils.gen_g(R_tf, des_loc) 
    
    # ---------------------------------------------------------------------------------------------------------------- #
    def proj_curve_to_line_obj_func(self, r, curve, pt, min_dist):
        # Ensure r is within bounds
        r = np.clip(r, 0, 1)
        proj_line = misc_utils.proj_curve_to_line(r, curve, pt)

        # Calculate distances between original curve points and projected line points
        distances = np.linalg.norm(curve - proj_line, axis=1)

        # Penalty for distances below the threshold
        penalty_factor = 1000
        penalties = np.where(distances > min_dist, penalty_factor * (distances - min_dist), 0)

        # Objective is to minimize the total distance, with penalties for violations
        return np.sum(distances) + np.sum(penalties)
    
    def optimize_ratio(self, init_r, curve, pt, min_dist):
        result = minimize(self.proj_curve_to_line_obj_func, init_r, args=(curve, pt, min_dist),
                        bounds=[(0, 1)], method='L-BFGS-B')
        optimal_r = result.x[0]
        return optimal_r
    
    def get_pull_dir_mag(self, ctrd_3d, bnd_3d, min_pull_dist):
        optimal_r = self.optimize_ratio(0.5, bnd_3d, ctrd_3d, min_pull_dist)
        proj_line = misc_utils.proj_curve_to_line(optimal_r, bnd_3d, ctrd_3d)
        pull_dirs = proj_line - bnd_3d
        pull_dirs_norm = pull_dirs / np.linalg.norm(pull_dirs, axis=1)[:, np.newaxis]
        avg_pull_dir = np.mean(pull_dirs_norm, axis=0)
        avg_pull_dir /= np.linalg.norm(avg_pull_dir)  # Ensure it's a unit vector
        pull_mags = np.linalg.norm(pull_dirs, axis=1)
        avg_pull_mag = np.mean(pull_mags)
        return avg_pull_dir, avg_pull_mag
    
    # ---------------------------------------------------------------------------------------------------------------- #

    def tf_align(self, g_armbase_armjaw, g_fbfjaw, bnd_3d, skel_3d, ctrd_3d, grasp_ratio=0.5, grasp_depth=0.003):
        new_g_fbfjaw, pca_comps = misc_utils.align_fbfjaw(ctrd_3d, bnd_3d, skel_3d, g_fbfjaw)
        self.ral.loginfo(f"new_g_fbfjaw: {new_g_fbfjaw}")
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        self.ral.loginfo(f"g_offset: {g_offset}")
        skel_bnd_vec = pca_comps[1]
        ctrd_bnd_vecs = bnd_3d - ctrd_3d
        bnd_intersect_idx = np.argmin([np.linalg.norm(ctrd_bnd_vec - skel_bnd_vec) for ctrd_bnd_vec in ctrd_bnd_vecs])
        grasp_3d = (bnd_3d[bnd_intersect_idx] + ctrd_3d)*grasp_ratio + pca_comps[2]*grasp_depth
        return g_armbase_armjaw.dot(g_offset), grasp_3d
    
    def tf_grasp(self, g_armbase_armjaw, g_fbfjaw, grasp_3d):
        new_g_fbfjaw = np.copy(g_fbfjaw)
        new_g_fbfjaw[:3,3] = grasp_3d
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        g_grasp = g_armbase_armjaw.dot(g_offset)
        self.ral.loginfo(f"g_offset: {g_offset}")
        return g_grasp
    
    def tf_pull(self, g_armbase_armjaw, g_fbfjaw, bnd_3d):
        new_g_fbfjaw = np.copy(g_fbfjaw)
        pull_dir, pull_dist = self.get_pull_dir_mag(g_fbfjaw[:3,3], bnd_3d, self.max_pull_dist)
        self.ral.loginfo(f"pull_dir: {pull_dir}, pull_dist: {pull_dist}")
        new_g_fbfjaw[:3,3] += pull_dir*pull_dist
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        g_pull = g_armbase_armjaw.dot(g_offset)
        return g_pull

    # def tf_pull(self, g_armbase_armjaw, g_fbfjaw, bnd_3d):
    #     new_g_fbfjaw = np.copy(g_fbfjaw)
    #     pull_dir, pull_dist = misc_utils.get_pull_dir_mag(g_fbfjaw, bnd_3d)
    #     self.ral.loginfo(f"pull_dir: {pull_dir}, pull_dist: {pull_dist}")
    #     new_g_fbfjaw[:3,3] += pull_dir*pull_dist
    #     g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
    #     g_pull = g_armbase_armjaw.dot(g_offset)
    #     return g_pull

    def dvrk_key_ctrl(self):
        while self.key != 't':
            """Process user key inputs in a timer-based loop."""
            if self.key == "a":
                if self.bnd_3d is None or self.ctrd_3d is None or self.g_fbfjaw is None:
                    self.ral.loginfo("Tissue has not been detected yet!")
                    continue
                self.ral.loginfo("Aligning the forceps before grasping...")
                g_align, self.grasp_3d = self.tf_align(
                    np.copy(self.custom_local_jaw_cp),
                    np.copy(self.g_fbfjaw),
                    np.copy(self.bnd_3d),
                    np.copy(self.skel_3d),
                    np.copy(self.ctrd_3d)
                )
                self.align_ik.target = g_align
                self.jp_align = self.align_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                self.ral.loginfo(f"jp_align: {self.jp_align}")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, math.radians(0), 1)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_align, 5)
                self.ral.loginfo("Finished aligning the forceps!")
                self.key = None

            elif self.key == "g":
                if self.jp_align is None:
                    self.ral.loginfo("Align the forceps before grasping!")
                    continue
                self.ral.loginfo("Start grasping the tissue...")
                g_grasp = self.tf_grasp(
                    np.copy(self.custom_local_jaw_cp),
                    np.copy(self.g_fbfjaw),
                    self.grasp_3d
                )
                self.move_ik.target = g_grasp
                self.jp_grasp = self.move_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, math.radians(self.jaw_open_angle), 2)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_grasp, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, math.radians(-10), 2)
                self.jp_release = np.copy(self.jp_grasp)
                self.ral.loginfo("Finished grasping the tissue!")
                self.key = None

            elif self.key == "p":
                if self.ctrd_3d is None or self.bnd_3d is None:
                    self.ral.loginfo("Tissue has not been detected yet!")
                    continue
                self.ral.loginfo("Pulling the tissue...")
                g_pull = self.tf_pull(
                    self.custom_local_jaw_cp,
                    self.g_fbfjaw, self.bnd_3d
                )
                self.move_ik.target = g_pull
                self.jp_pull = self.move_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_pull, 5)
                self.ral.loginfo("Finished pulling the tissue!")
                self.key = None

            elif self.key == "r":
                if self.jp_grasp is None:
                    self.ral.loginfo("The arm didn't grasp the tissue yet!")
                    continue
                self.ral.loginfo("Start releasing the tissue...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_release, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval,math.radians(self.jaw_open_angle), 2)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_align, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval,math.radians(-10), 2)
                self.reset()
                self.ral.loginfo("Finished releasing the tissue!")
                self.key = None

            elif self.key == "o":
                self.ral.loginfo("Opening the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval,math.radians(self.jaw_open_angle), 2)
                self.ral.loginfo("The jaws are opened!")
                self.key = None

            elif self.key == "c":
                self.ral.loginfo("Closing the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval,math.radians(-10), 2)
                self.ral.loginfo("The jaws are closed!")
                self.key = None

            elif self.key == "i":
                self.ral.loginfo("Moving to initial position...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.init_jp, 5)
                self.ral.loginfo("Moved to initial position!")
                self.key = None

    def run(self):
        self.add_arm_ik()
        self.add_subs()
        self.dvrk_key_ctrl()
    