# ROS2 Libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped
from std_msgs.msg import String

# Python Libraries
import math
import time
import numpy as np
from scipy.optimize import minimize
from sklearn.decomposition import PCA

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
        self.align_dist      = params["align_dist"]
        self.grasp_offset    = params["grasp_offset"]
        self.align_ratio     = params["align_ratio"]
        self.jaw_open_angle  = math.radians(params["jaw_open_angle"])
        self.jaw_close_angle = math.radians(params["jaw_close_angle"])
        self.min_pull_dist   = params["min_pull_dist"]
        self.max_pull_dist   = params["max_pull_dist"]
        self.grasp_depth     = params["grasp_depth"]
        self.open_close_dur  = params["open_close_dur"]

    def add_arm_ik(self):
        self.init_jp = dvrk_utils.get_jp(self.arm)
        self.ral.loginfo(f"init_jp: {self.init_jp}")
        self.arm_ik = ik_utils.dvrk_custom_ik(
            self.params["calib_fn"],
            self.params["arm_ik_W"][0],
            self.params["arm_ik_W"][1],
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
    
    # ----------------------------------------------------------------------------------------------------------------

    def filt_bnd_pts(self, curve_points):
        pca = PCA(n_components=3)
        pca.fit(curve_points)

        main_axis = pca.components_[0]  # First principal component

        projected_points = np.dot(curve_points - np.mean(curve_points, axis=0), main_axis)

        threshold = np.percentile(np.abs(projected_points), 90)
        selected_points = curve_points[np.abs(projected_points) <= threshold]

        return selected_points

    def get_pull_dir_mag(self, bnd_3d):
        """
        Computes the deviation of a 3D curve from the reference line connecting its first and last points.
        :param points: (N, 3) array of 3D points representing the curve.
        :return: deviations (N,) array of distances to the reference line, and the average deviation vector.
        """
        P1 = bnd_3d.mean(axis=0)

        # Direction vector of the reference line
        d_unit = misc_utils.cnt_axes_3d(bnd_3d)[0]

        # Compute projections and deviations
        deviation_vectors = []
        deviations = []

        for Pi in bnd_3d:
            ti = np.dot(Pi - P1, d_unit)  # Projection scalar
            Qi = P1 + ti * d_unit  # Closest point on the reference line
            deviation_vector = Qi - Pi  # Vector from input points to the projected line
            deviation_magnitude = np.linalg.norm(deviation_vector)  # Distance to the line
            deviations.append(deviation_magnitude)
            if deviation_magnitude != 0:
                deviation_vectors.append(deviation_vector / deviation_magnitude)

        deviations = np.array(deviations)
        deviation_vectors = np.vstack(deviation_vectors)
        return np.mean(deviation_vectors, axis=0), np.mean(deviations)

    # def get_pull_dir_mag(self, bnd_3d, pull_dir):
    #     bnd_center = misc_utils.midpt_curve(bnd_3d)

    #     # Compute projections and deviations
    #     deviations = []

    #     for Pi in bnd_3d:
    #         proj_mag = np.dot(Pi - bnd_center, pull_dir)
    #         deviations.append(proj_mag)

    #     deviations = np.array(deviations)
    #     return np.mean(deviations)
    
    # ---------------------------------------------------------------------------------------------------------------- #

    def tf_align(self, g_armbase_armjaw, g_fbfjaw, bnd_3d, skel_3d, ctrd_3d, grasp_depth=0.015):
        new_g_fbfjaw, bnd_normal = misc_utils.align_fbfjaw(ctrd_3d, bnd_3d, skel_3d, g_fbfjaw)
        self.ral.loginfo(f"new_g_fbfjaw: {new_g_fbfjaw}")
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        self.ral.loginfo(f"g_offset: {g_offset}")
        # bnd_pca_comps = misc_utils.cnt_axes_3d(bnd_3d)
        bnd_center = misc_utils.midpt_curve(bnd_3d)
        grasp_3d = (bnd_center + ctrd_3d)/2 - bnd_normal*grasp_depth
        return g_armbase_armjaw.dot(g_offset), grasp_3d
    
    def tf_grasp(self, g_armbase_armjaw, g_fbfjaw, grasp_3d):
        new_g_fbfjaw = np.copy(g_fbfjaw)
        new_g_fbfjaw[:3,3] = grasp_3d
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        g_grasp = g_armbase_armjaw.dot(g_offset)
        self.ral.loginfo(f"g_offset: {g_offset}")
        return g_grasp
    
    # def tf_pull(self, g_armbase_armjaw, g_fbfjaw, bnd_3d):
    #     new_g_fbfjaw = np.copy(g_fbfjaw)
    #     pull_dir, pull_dist = self.get_pull_dir_mag(g_fbfjaw[:3,3], bnd_3d, self.max_pull_dist)
    #     self.ral.loginfo(f"pull_dir: {pull_dir}, pull_dist: {pull_dist}")
    #     new_g_fbfjaw[:3,3] += pull_dir*pull_dist
    #     g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
    #     g_pull = g_armbase_armjaw.dot(g_offset)
    #     return g_pull

    def tf_pull(self, g_armbase_armjaw, g_fbfjaw, pull_dir, pull_dist):
        new_g_fbfjaw = np.copy(g_fbfjaw)
        new_g_fbfjaw[:3,3] += pull_dir*pull_dist
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        g_pull = g_armbase_armjaw.dot(g_offset)
        return g_pull

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
                    np.copy(self.ctrd_3d),
                    self.grasp_depth
                )
                self.arm_ik.target = g_align
                self.jp_align = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                self.ral.loginfo(f"jp_align: {self.jp_align}")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, self.open_close_dur)
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
                self.arm_ik.target = g_grasp
                self.jp_grasp = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, self.open_close_dur)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_grasp, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, self.open_close_dur)
                self.jp_release = np.copy(self.jp_grasp)
                self.ral.loginfo("Finished grasping the tissue!")
                self.key = None

            elif self.key == "p":
                while self.key != 's':
                    if self.bnd_3d is None:
                        self.ral.loginfo("Tissue has not been detected yet!")
                        continue
                    # bnd_3d = self.filt_bnd_pts(
                    #     np.copy(self.bnd_3d),
                    #     np.copy(self.g_fbfjaw[:3, 3]),
                    #     60
                    # )
                    bnd_3d = self.filt_bnd_pts(
                        np.copy(self.bnd_3d)
                    )
                    # bnd_3d = np.copy(self.bnd_3d)
                    g_fbfjaw = np.copy(self.g_fbfjaw)
                    pull_dir, pull_dist = self.get_pull_dir_mag(bnd_3d)
                    # pull_dist = self.get_pull_dir_mag(bnd_3d, -g_fbfjaw[:3,2])
                    self.ral.loginfo(f"pull_dist: {pull_dist}")
                    if pull_dist > self.max_pull_dist:
                        continue
                    if pull_dist < self.min_pull_dist:
                        self.key = None
                        break
                    self.ral.loginfo("Pulling the tissue...")
                    g_pull = self.tf_pull(
                        np.copy(self.custom_local_jaw_cp),
                        g_fbfjaw,
                        -g_fbfjaw[:3,2], pull_dist
                    )
                    # g_pull = self.tf_pull(
                    #     np.copy(self.custom_local_jaw_cp),
                    #     g_fbfjaw,
                    #     pull_dir, pull_dist
                    # )
                    self.arm_ik.target = g_pull
                    self.jp_pull = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                    dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_pull, 0.1)
                    time.sleep(2)
                self.ral.loginfo("Finished pulling the tissue!")

            elif self.key == "r":
                if self.jp_grasp is None:
                    self.ral.loginfo("The arm didn't grasp the tissue yet!")
                    continue
                self.ral.loginfo("Start releasing the tissue...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_release, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, self.open_close_dur)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_align, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, self.open_close_dur)
                self.reset()
                self.ral.loginfo("Finished releasing the tissue!")
                self.key = None

            elif self.key == "o":
                self.ral.loginfo("Opening the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, self.open_close_dur)
                self.ral.loginfo("The jaws are opened!")
                self.key = None

            elif self.key == "c":
                self.ral.loginfo("Closing the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, self.open_close_dur)
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
    