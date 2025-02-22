# ROS2 Libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped
from std_msgs.msg import String

# Python Libraries
import math
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
    
    # ---------------------------------------------------------------------------------------------------------------- #
    # def curve_length(self, curve):
    #     return np.sum(np.linalg.norm(curve[1:] - curve[:-1], axis=1))

    # def proj_curve_to_line(self, r, curve, pt):
    #     curve_len = self.curve_length(curve)
    #     curve2pt_vecs = pt - curve
    #     mid_pt_vecs = curve + curve2pt_vecs*r
    #     mid_pt = np.mean(mid_pt_vecs, axis=0)
        
    #     # Calculate the direction vector for the stretched line as before
    #     endpt_vec = curve[-1] - curve[0]
    #     endpt_vec /= np.linalg.norm(endpt_vec)
        
    #     # Use the new midpoint and the same direction to define the stretched line
    #     return np.linspace(
    #         mid_pt - endpt_vec * curve_len / 2,
    #         mid_pt + endpt_vec * curve_len / 2,
    #         curve.shape[0]
    #     )

    # def proj_curve_to_line_obj_func(self, r, curve, pt, min_dist):
    #     # Ensure r is within bounds
    #     r = np.clip(r, 0, 1)
    #     proj_line = self.proj_curve_to_line(r, curve, pt)

    #     # Calculate distances between original curve points and projected line points
    #     distances = np.linalg.norm(curve - proj_line, axis=1)

    #     # Penalty for distances below the threshold
    #     penalty_factor = 1000
    #     penalties = np.where(distances > min_dist, penalty_factor * (distances - min_dist), 0)

    #     # Objective is to minimize the total distance, with penalties for violations
    #     return np.sum(distances) + np.sum(penalties)
    
    # def optimize_ratio(self, init_r, curve, pt, min_dist):
    #     result = minimize(self.proj_curve_to_line_obj_func, init_r, args=(curve, pt, min_dist),
    #                     bounds=[(0, 1)], method='L-BFGS-B')
    #     optimal_r = result.x[0]
    #     return optimal_r
    
    # def get_pull_dir_mag(self, ctrd_3d, bnd_3d, min_pull_dist):
    #     optimal_r = self.optimize_ratio(0.5, bnd_3d, ctrd_3d, min_pull_dist)
    #     proj_line = misc_utils.proj_curve_to_line(optimal_r, bnd_3d, ctrd_3d)
    #     pull_dirs = proj_line - bnd_3d
    #     pull_dirs_norm = pull_dirs / np.linalg.norm(pull_dirs, axis=1)[:, np.newaxis]
    #     avg_pull_dir = np.mean(pull_dirs_norm, axis=0)
    #     avg_pull_dir /= np.linalg.norm(avg_pull_dir)  # Ensure it's a unit vector
    #     pull_mags = np.linalg.norm(pull_dirs, axis=1)
    #     avg_pull_mag = np.mean(pull_mags)
    #     return avg_pull_dir, avg_pull_mag

    def filt_bnd_pts(self, curve_points):
        pca = PCA(n_components=3)
        pca.fit(curve_points)

        main_axis = pca.components_[0]  # First principal component

        projected_points = np.dot(curve_points - np.mean(curve_points, axis=0), main_axis)

        threshold = np.percentile(np.abs(projected_points), 80)
        selected_points = curve_points[np.abs(projected_points) <= threshold]

        return selected_points

    # def filt_bnd_pts(self, points, ref_point, max_angle_deg):
    #     distances = np.linalg.norm(points - ref_point, axis=1)
    #     closest_point = points[np.argmin(distances)]

    #     # Reference direction: from ref_point to closest_point, normalized
    #     ref_dir = closest_point - ref_point
    #     norm_ref_dir = np.linalg.norm(ref_dir)
    #     if norm_ref_dir == 0:
    #         raise ValueError("Reference point is exactly on the curve; undefined reference direction.")
    #     ref_dir /= norm_ref_dir

    #     max_angle_rad = np.radians(max_angle_deg)

    #     # Compute unit direction from ref_point to all points
    #     directions = points - ref_point
    #     norms = np.linalg.norm(directions, axis=1)
    #     # Avoid division by zero; set zero norms to 1 (their corresponding points equal ref_point)
    #     norms[norms == 0] = 1.0
    #     unit_dirs = directions / norms[:, None]

    #     # Compute angles using the dot product (clip for safety)
    #     cos_angles = np.clip(np.dot(unit_dirs, ref_dir), -1.0, 1.0)
    #     angles = np.arccos(cos_angles)

    #     # Filter points based on the max angle threshold
    #     mask = angles <= max_angle_rad
    #     return points[mask]

    def get_pull_dir_mag(self, bnd_3d):
        """
        Computes the deviation of a 3D curve from the reference line connecting its first and last points.
        :param points: (N, 3) array of 3D points representing the curve.
        :return: deviations (N,) array of distances to the reference line, and the average deviation vector.
        """
        P1 = bnd_3d[0]  # First point

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
            deviation_vectors.append(deviation_vector / deviation_magnitude if deviation_magnitude != 0 else deviation_vector)

        deviations = np.array(deviations)
        deviation_vectors = np.vstack(deviation_vectors)
        return np.mean(deviation_vectors, axis=0), np.mean(deviations)
    
    # ---------------------------------------------------------------------------------------------------------------- #

    def tf_align(self, g_armbase_armjaw, g_fbfjaw, bnd_3d, skel_3d, ctrd_3d, grasp_ratio=0.7, grasp_depth=0.01):
        new_g_fbfjaw, bnd_normal = misc_utils.align_fbfjaw(ctrd_3d, bnd_3d, skel_3d, g_fbfjaw)
        self.ral.loginfo(f"new_g_fbfjaw: {new_g_fbfjaw}")
        g_offset = tf_utils.ginv(g_fbfjaw).dot(new_g_fbfjaw)
        self.ral.loginfo(f"g_offset: {g_offset}")
        # bnd_pca_comps = misc_utils.cnt_axes_3d(bnd_3d)
        bnd_center = misc_utils.midpt_curve(bnd_3d)
        grasp_3d = (bnd_center + ctrd_3d)/2 - bnd_normal*0.01
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
                    np.copy(self.ctrd_3d)
                )
                self.arm_ik.target = g_align
                self.jp_align = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                self.ral.loginfo(f"jp_align: {self.jp_align}")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, 1)
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
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, 1)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_grasp, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, 1)
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
                    _, pull_dist = self.get_pull_dir_mag(bnd_3d)
                    # if pull_dist > self.max_pull_dist:
                    #     continue
                    if pull_dist < self.min_pull_dist:
                        self.key = None
                        break
                    self.ral.loginfo("Pulling the tissue...")
                    self.ral.loginfo(f"pull_dist: {pull_dist}")
                    g_fbfjaw = np.copy(self.g_fbfjaw)
                    g_pull = self.tf_pull(
                        np.copy(self.custom_local_jaw_cp),
                        g_fbfjaw,
                        -g_fbfjaw[:3,2], pull_dist
                    )
                    self.arm_ik.target = g_pull
                    self.jp_pull = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                    dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_pull, 1)
                    for i in range(200):
                        self.sleep_rate.sleep()
                self.ral.loginfo("Finished pulling the tissue!")

            elif self.key == "r":
                if self.jp_grasp is None:
                    self.ral.loginfo("The arm didn't grasp the tissue yet!")
                    continue
                self.ral.loginfo("Start releasing the tissue...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_release, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, 1)
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_align, 5)
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, 1)
                self.reset()
                self.ral.loginfo("Finished releasing the tissue!")
                self.key = None

            elif self.key == "o":
                self.ral.loginfo("Opening the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_open_angle, 1)
                self.ral.loginfo("The jaws are opened!")
                self.key = None

            elif self.key == "c":
                self.ral.loginfo("Closing the jaw...")
                dvrk_utils.run_jaw_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jaw_close_angle, 1)
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
    