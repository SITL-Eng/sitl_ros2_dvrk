# ROS2 Libraries
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, PointStamped, PoseStamped

# Python Libraries
import math
import numpy as np
from scipy.spatial import KDTree

# dVRK
import dvrk

# Custom Libraries 
from sitl_ros2_interfaces.msg import StringStamped, Dt2KptState, BoolStamped
from utils import tf_utils, pcl_utils, misc_utils, ik_utils, dvrk_utils, ma_utils

class DISSECT:
    def __init__(self, ral, params):
        self.params    = params
        self.ral       = ral
        self.key       = None
        self.bnd_3d    = None
        self.skel_3d   = None
        self.g_pchjaw  = None
        self.grasp_dir = None
        self.jp_align  = None
        self.pch_angle_offset = misc_utils.law_of_cos_ang(0.0049, 0.0036, 0.0018)
        self.custom_arm_cp       = None
        self.custom_local_arm_cp = None
        self.custom_jaw_cp       = None
        self.custom_local_jaw_cp = None
        
        self.expected_interval = params['expected_interval']
        self.arm_cp_tn = "/{}/custom/setpoint_cp".format(params['arm_name'])
        self.arm_local_cp_tn = "/{}/custom/local/setpoint_cp".format(params['arm_name'])
        self.jaw_cp_tn = "/{}/custom/jaw/setpoint_cp".format(params['arm_name'])
        self.jaw_local_cp_tn = "/{}/custom/local/jaw/setpoint_cp".format(params['arm_name'])
        self.arm = dvrk.psm(
            ral = ral,
            arm_name = params['arm_name'],
            expected_interval = self.expected_interval
        )
        self.sleep_rate = self.ral.create_rate(1.0 / self.expected_interval)
        self.g_ecmopencv_ecmdvrk = self.load_tf(params["tf_path"])
        self.pedal_flag = params["pedal_flag"]
        self.pedal_mp_msg = BoolStamped()
        self.pedal_mp_pub = self.ral.publisher("/pedal/monopolar/write", BoolStamped, queue_size=10)
        self.pub_cur_traj = self.ral.publisher("/target", PointStamped, queue_size=10)
        self.cur_idx = 0
        self.success = 0
        self.d_eps = params['d_eps']
        self.move_dist = params["move_dist"]

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

    def pch_jaw_kpt_cb(self, pch_jaw_cp_msg):
        self.g_pchjaw = tf_utils.tfstamped2g(pch_jaw_cp_msg)
    
    def add_subs(self):
        self.custom_arm_cp_sub = self.ral.subscriber(
            self.arm_cp_tn,
            PoseStamped,
            self.custom_arm_cp_callback,
        )
        self.custom_arm_local_cp_sub = self.ral.subscriber(
            self.arm_local_cp_tn,
            PoseStamped,
            self.custom_local_arm_cp_callback,
        )
        self.custom_jaw_cp_sub = self.ral.subscriber(
            self.jaw_cp_tn,
            PoseStamped,
            self.custom_jaw_cp_callback,
        )
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
        self.bnd_sub = self.ral.subscriber(
            self.params['bnd_topic'],
            PointCloud2,
            self.bnd_3d_cb,
            self.params['queue_size']
        )
        self.ral.subscriber(
            self.params['skel_topic'],
            PointCloud2,
            self.skel_3d_cb,
            self.params['queue_size']
        )
        self.pch_jaw_kpt_sub = self.ral.subscriber(
            self.params['pch_jaw_kpt_topic'],
            TransformStamped,
            self.pch_jaw_kpt_cb,
            self.params['queue_size']
        )
    
    def reset(self):
        self.bnd_3d   = None
        self.kpt_nms  = None
        self.pch_3d   = None
        self.g_pchjaw = None
        self.align_jp = None

    def load_tf(self, tf_path):
        tf_data = tf_utils.load_tf_data(tf_path)
        g_ecmdvrk_ecmopencv = np.array(tf_data["g_ecmdvrk_ecmopencv"])
        return tf_utils.ginv(g_ecmdvrk_ecmopencv)
    
    # def tf_align_pch(self, g_ecmtip_armtip, g_armbase_armtip, kpt_nms, pch_3d, g_pchjaw, bnd_3d):
    #     if "CentralHook" not in kpt_nms:
    #         self.ral.loginfo("Missing Keypoints...")
    #         return None
    #     g_armtip_ecmopencv = tf_utils.ginv(self.g_ecmopencv_ecmdvrk.dot(g_ecmtip_armtip))
    #     v_ch_th = g_pchjaw[:3,3] - pch_3d[kpt_nms.index("CentralHook")]
    #     v_bs_bf = bnd_3d[-1] - bnd_3d[0]
    #     angle = misc_utils.angle_btw_vecs(v_ch_th, v_bs_bf)
    #     w_rot = misc_utils.unit_vector(np.cross(v_ch_th, v_bs_bf))
    #     w_rot = tf_utils.gdotv(g_armtip_ecmopencv, w_rot)
    #     if w_rot[0] > 0:
    #         angle -= self.pch_angle_offset
    #     else:
    #         angle += self.pch_angle_offset
    #     return g_armbase_armtip.dot(tf_utils.cv2vecs2g(w_rot*angle, np.array([0,0,0])))

    def tf_align(self, g_armbase_armjaw, g_pchjaw, bnd_3d, skel_3d):
        new_g_pchjaw = misc_utils.align_pchjaw(bnd_3d, skel_3d, g_pchjaw)
        self.ral.loginfo(f"new_g_pchjaw: {new_g_pchjaw}")
        g_offset = tf_utils.ginv(g_pchjaw).dot(new_g_pchjaw)
        self.ral.loginfo(f"g_offset: {g_offset}")
        return g_armbase_armjaw.dot(g_offset)
    
    def get_cur_traj_pt(self, prev_traj_pt, bnd_3d, g_pchjaw, d_thr=9e-3, ang_thr=45):
        cur_traj_pt = np.copy(prev_traj_pt)
        th = g_pchjaw[:3,3]
        end_flag = False
        max_dist = 0
        bnd_3d_dir = misc_utils.cnt_axes_3d(bnd_3d)[0]
        traj_tree = KDTree(bnd_3d)
        matching_indices = traj_tree.query_ball_point(th, d_thr)
        if matching_indices:
            for idx in matching_indices:
                dist = np.linalg.norm(bnd_3d[idx] - th)
                angle = math.degrees(misc_utils.angle_btw_vecs(bnd_3d_dir, bnd_3d[idx]-th))
                if dist > max_dist and abs(angle) < ang_thr:
                    max_dist = dist
                    cur_traj_pt = bnd_3d[idx]
        if np.linalg.norm(th - cur_traj_pt) < self.d_eps:
            end_flag = True
        return end_flag, cur_traj_pt

    def dvrk_key_ctrl(self):
        while self.key != 't':
            """Process user key inputs in a timer-based loop."""
            if self.key == "a":
                if self.bnd_3d is None or self.g_pchjaw is None:
                    self.ral.loginfo("Tissue has not been detected yet!")
                    continue
                self.ral.loginfo("Aligning the arm before following boundary...")
                self.arm_ik.target = self.tf_align(
                    np.copy(self.custom_local_jaw_cp),
                    np.copy(self.g_pchjaw),
                    np.copy(self.bnd_3d),
                    np.copy(self.skel_3d)
                )
                if self.arm_ik.target is None:
                    continue
                self.jp_align = self.arm_ik.get_goal_jp_jaw(dvrk_utils.get_jp(self.arm))
                self.ral.loginfo(f"jp_align: {self.jp_align}")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval, self.jp_align, 5)
                self.ral.loginfo("Finished aligning the hook!")
                self.key = None
            elif self.key == "f":
                cur_traj_pt = None
                self.ral.loginfo("Start following the tissue boundary...")
                self.ral.loginfo("Press b if you want to terminate...")
                while True:
                    if 'b' in self.key:
                        self.ral.loginfo("Terminating...")
                        break
                    if self.bnd_3d is None:
                        continue
                    bnd_3d = np.copy(self.bnd_3d)
                    g_pchjaw = np.copy(self.g_pchjaw)
                    if cur_traj_pt is None:
                        init_flag = True
                        cur_traj_pt = bnd_3d[0]
                    if np.linalg.norm(cur_traj_pt - g_pchjaw[:3,3]) < self.d_eps:
                        if init_flag:
                            init_flag = False
                        end_flag, cur_traj_pt = self.get_cur_traj_pt(
                            cur_traj_pt, bnd_3d, g_pchjaw
                        )
                        if end_flag:
                            self.ral.loginfo("Finished Dissection...")
                            if self.jp_align is None:
                                dvrk_utils.run_arm_servo_jp(
                                    self.arm, self.sleep_rate,
                                    self.expected_interval,
                                    self.init_jp, 3
                                )
                            else:
                                dvrk_utils.run_arm_servo_jp(
                                    self.arm, self.sleep_rate, 
                                    self.expected_interval,
                                    self.jp_align, 3
                                )
                            break
                    cur_traj_msg = tf_utils.pt3d2ptstamped(
                        cur_traj_pt, self.ral.now().to_msg(), "ecm_left"
                    )
                    self.pub_cur_traj.publish(cur_traj_msg)
                    g_armbase_ecmopencv = self.custom_local_arm_cp.dot(
                        tf_utils.ginv(self.g_ecmopencv_ecmdvrk.dot(self.custom_arm_cp))
                    )
                    self.arm_ik.target = self.custom_local_arm_cp
                    self.arm_ik.target[:3,3] += tf_utils.gdotv(
                        g_armbase_ecmopencv,
                        misc_utils.unit_vector(
                            cur_traj_pt - g_pchjaw[:3,3]
                        )*self.move_dist
                    )
                    if not init_flag and self.pedal_flag:
                        self.pedal_mp_msg.header.stamp = self.ral.now().to_msg()
                        self.pedal_mp_msg.data = True
                        self.pedal_mp_pub.publish(self.pedal_mp_msg)
                    dvrk_utils.run_arm_servo_jp(
                        self.arm, self.sleep_rate, self.expected_interval,
                        self.arm_ik.get_goal_jp(dvrk_utils.get_jp(self.arm)), 0.2
                    )
                    if not init_flag and self.pedal_flag:
                        self.pedal_mp_msg.header.stamp = self.ral.now().to_msg()
                        self.pedal_mp_msg.data = False
                        self.pedal_mp_pub.publish(self.pedal_mp_msg)
                self.key = None

            elif self.key == "r":
                self.ral.loginfo("Moving to the position before following the boundary...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.jp_align, 5)
                self.reset()
                self.ral.loginfo("Finished moving the arm!")
                self.key = None

            elif self.key == "i":
                self.ral.loginfo("Moving to its initial position...")
                dvrk_utils.run_arm_servo_jp(self.arm, self.sleep_rate, self.expected_interval,self.init_jp, 5)
                self.ral.loginfo("Moved to initial position!")
                self.key = None

    def run(self):
        self.add_arm_ik()
        self.add_subs()
        self.dvrk_key_ctrl()
    