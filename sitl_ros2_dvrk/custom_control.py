#!/usr/bin/env python3

import sys
import dvrk
import crtk
import rclpy
from rclpy.node import Node
import numpy as np
np.seterr(all="ignore")
import math
from geometry_msgs.msg import PoseStamped
from utils import ik_utils, tf_utils

# example of application using arm.py
class DVRK_CTRL:

    # configuration
    def __init__(self, arm_name, expected_interval):
        self.arm_name = arm_name
        # super().__init__(node_name)
        # print(self, 'configuring dvrk_arm_test for %s' % arm_name)
        self.ral = crtk.ral(self.arm_name) # node is created inside the ral as ral._node obejct
        self.expected_interval = expected_interval
        self.sleep_rate = self.ral.create_rate(1.0 / self.expected_interval)



        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._node)

    
        # Create a subscriber for the arm's cartesian position
        # self.custom_arm_cp_subscriber = ral.subscriber(
        #     self.arm_cp_tn,
        #     PoseStamped,
        #     self.custom_arm_cp_callback,
        # )

        self.arm = dvrk.arm(
            ral = self.ral,
            arm_name = self.arm_name,
            expected_interval = expected_interval
        )
        # self.arm.check_connections()
        # print(np.copy(self.arm.setpoint_jp()))

        # qos = custom_qos_profile(10)
        # self.custom_arm_cp_subscriber = ral._node.create_subscription(PoseStamped, self.arm_cp_tn,
        #                                     self.custom_arm_cp_callback, qos)
        # print(np.copy(self.arm.setpoint_jp()))

        # self.custom_arm_cp_subscriber = self._node.create_subscription(
        #     msg_type, self._add_namespace_to(topic),
        #                                       callback, qos, *args, **kwargs)

        # Create a subscriber for the arm's local cartesian position
        # self.custom_arm_local_cp_subscriber = ral.subscriber(
        #     self.arm_local_cp_tn,
        #     PoseStamped,
        #     self.custom_local_arm_cp_callback,
        # )

        # # Create a subscriber for the jaw's cartesian position (if applicable)
        # if hasattr(self, 'jaw_cp_tn'):
        #     self.custom_jaw_cp_subscriber = ral.subscriber(
        #         self.jaw_cp_tn,
        #         PoseStamped,
        #         self.custom_jaw_cp_callback,
        #     )

        # # Create a subscriber for the jaw's local cartesian position (if applicable)
        # if hasattr(self, 'jaw_local_cp_tn'):
        #     self.custom_jaw_local_cp_subscriber = ral.subscriber(
        #         self.jaw_local_cp_tn,
        #         PoseStamped,
        #         self.custom_local_jaw_cp_callback,
        #     )


        # if "PSM" in arm_name:
        #     self.jaw_cp_tn = "/{}/custom/jaw/setpoint_cp".format(arm_name)
        #     self.jaw_local_cp_tn = "/{}/custom/local/jaw/setpoint_cp".format(arm_name)
        #     self.arm = dvrk.psm(
        #         ral = self.ral,
        #         arm_name = arm_name,
        #         expected_interval = expected_interval
        #     )
        # elif "MTM" in arm_name:
        #     self.arm = dvrk.mtm(
        #         ral = self.ral,
        #         arm_name = arm_name,
        #         expected_interval = expected_interval
        #     )
        #     self.arm.jaw = self.arm.gripper
        # else:
        #     self.arm = dvrk.arm(
        #         ral = self.ral,
        #         arm_name = arm_name,
        #         expected_interval = expected_interval
        #     )
        
        

        
        
        
        # print("getting initial joint position")
        # self.init_jp = np.copy(self.get_jp())
        # print("initial joint position: ", self.init_jp)

    # def __del__(self):
    #     print(self, "Destructing Class DVRK_CTRL...")

    def init_ik(self):
        # IK module
        self.arm_ik = ik_utils.dvrk_custom_ik(
            calib_fn = "/home/hossein/aruco_data/psm1_calib_results_final_new_v2.mat",
            wT = 1, wR = 0.1, init_jp = np.copy(self.arm.setpoint_jp()), Joffsets = np.array([30, 30, 5, 60, 80, 80])
        )

    def init_custom_cp_subscribers(self):
        self.arm_cp_tn = "/{}/custom/setpoint_cp".format(self.arm_name)
        self.arm_local_cp_tn = "/{}/custom/local/setpoint_cp".format(self.arm_name)
        self.jaw_cp_tn = "/{}/jaw/custom/setpoint_cp".format(self.arm_name)
        self.jaw_local_cp_tn = "/{}/jaw/custom/local/setpoint_cp".format(self.arm_name)
        # Initialize the custom arm cartesian position
        self.custom_arm_cp = None
        self.custom_local_arm_cp = None
        self.custom_jaw_cp = None
        self.custom_local_jaw_cp = None

        # Create a subscriber for the arm's cartesian position
        self.custom_arm_cp_subscriber = self.ral.subscriber(
            self.arm_cp_tn,
            PoseStamped,
            self.custom_arm_cp_callback,
        )

        # Create a subscriber for the arm's local cartesian position
        self.custom_arm_local_cp_subscriber = self.ral.subscriber(
            self.arm_local_cp_tn,
            PoseStamped,
            self.custom_local_arm_cp_callback,
        )

        # Create a subscriber for the jaw's cartesian position (if applicable)
        if hasattr(self, 'jaw_cp_tn'):
            self.custom_jaw_cp_subscriber = self.ral.subscriber(
                self.jaw_cp_tn,
                PoseStamped,
                self.custom_jaw_cp_callback,
            )

        # Create a subscriber for the jaw's local cartesian position (if applicable)
        if hasattr(self, 'jaw_local_cp_tn'):
            self.custom_jaw_local_cp_subscriber = self.ral.subscriber(
                self.jaw_local_cp_tn,
                PoseStamped,
                self.custom_local_jaw_cp_callback,
            )


        # if "PSM" in self.arm_name:
        #     self.jaw_cp_tn = "/{}/custom/jaw/setpoint_cp".format(arm_name)
        #     self.jaw_local_cp_tn = "/{}/custom/local/jaw/setpoint_cp".format(arm_name)
        #     self.arm = dvrk.psm(
        #         ral = self.ral,
        #         arm_name = arm_name,
        #         expected_interval = expected_interval
        #     )
        
    def home_zero_position(self, duration):
        print(self, 'starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print(self, 'starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print(self, 'move to zero position')
        zero_jp = np.copy(self.get_jp())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        zero_jp.fill(0)
        if "PSM" in self.arm.name():
            self.arm.jaw.open(angle=math.radians(0)).wait()
        else:
            zero_jp[2] = 0.03
        self.run_arm_servo_jp(zero_jp, duration)
        print(self, 'moving to zero position complete')

    def home_init_position(self, duration):
        # get current joints just to set size
        print(self, 'move to init position')
        self.run_arm_servo_jp(self.init_jp, duration)
        print(self, 'moving to init position complete')

    def home_test_position(self, duration):
        print(self, 'starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print(self, 'starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print(self, 'move to test position')
        zero_jp = np.copy(self.get_jp())
        # go to initial position
        zero_jp.fill(0)
        if "PSM" in self.arm.name():
            self.arm.jaw.open(angle=math.radians(0)).wait()
        zero_jp[2] = 0.12
        self.run_arm_servo_jp(zero_jp, duration)
        print(self, 'moving to test position complete')

    def update_init_jp(self):
        self.init_jp = np.copy(self.get_jp())

    # direct joint control example
    def run_arm_servo_jp(self, goal, duration=5):
        initial_joint_position = np.copy(self.get_jp())
        samples = duration / self.expected_interval
        amplitude = (goal - initial_joint_position) / samples
        for i in range(int(self.custom_arm_cp)):
            cur_goal = initial_joint_position + i * amplitude
            self.arm.servo_jp(cur_goal, amplitude)
            self.sleep_rate.sleep()

    def run_jaw_servo_jp(self, goal, duration=5):
        initial_joint_position = np.copy(self.get_jaw_jp())
        samples = duration / self.expected_interval
        amplitude = (goal - initial_joint_position) / samples
        for i in range(int(samples)):
            cur_goal = initial_joint_position + i * amplitude
            self.arm.jaw.servo_jp(cur_goal)
            self.sleep_rate.sleep()

    def run_full_servo_jp(self, arm_goal, jaw_goal=math.radians(0), duration=5):
        arm_init_jp = np.copy(self.get_jp())
        jaw_init_jp = np.copy(self.get_jaw_jp())
        samples = duration / self.expected_interval
        arm_amp = (arm_goal - arm_init_jp) / samples
        jaw_amp = (jaw_goal - jaw_init_jp) / samples
        for i in range(int(samples)):
            cur_arm_goal = arm_init_jp + i * arm_amp
            cur_jaw_goal = jaw_init_jp + i * jaw_amp
            self.arm.servo_jp(cur_arm_goal)
            self.arm.jaw.servo_jp(cur_jaw_goal)
            self.sleep_rate.sleep()
            # rclpy.spin_once(self)

    def test(self):
        #Get the current cartesian position
        current_cp = self.get_cp()
        print(self, f"Current CP: {current_cp}")
        
        # Define a new point slightly offset from the current position
        new_cp = np.copy(current_cp)
        new_cp[0:3, 3] += np.array([0.02, 0.02, 0.02])  # Move 2cm in x, y, and z directions

        # Get goal joint position
        self.arm_ik.target = new_cp
        goal_jp = self.arm_ik.get_goal_jp(self.arm.measured_jp())

        # Move to the new point
        print(self, f"Moving to new CP: {new_cp}")
        self.arm.move_jp(goal_jp).wait()
        print(self, "Move completed")

        # Verify the new position
        final_cp = self.custom_arm_cp
        print(self, f"Final CP: {final_cp}")

    
    # homing example
    def home(self):
        self.arm.check_connections()

        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print('move to starting position')
        goal = np.copy(self.arm.setpoint_jp())
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.03
        # move and wait
        print('moving to starting position')
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print('testing move to current position')
        move_handle = self.arm.move_jp(goal)
        print('move handle should return immediately')
        move_handle.wait()
        print('home complete')


    def custom_arm_cp_callback(self, msg):
        self.custom_arm_cp = tf_utils.posestamped2g(msg)

    def custom_local_arm_cp_callback(self, msg):
        self.custom_local_arm_cp = tf_utils.posestamped2g(msg)

    def custom_jaw_cp_callback(self, msg):
        self.custom_jaw_cp = tf_utils.posestamped2g(msg)

    def custom_local_jaw_cp_callback(self, msg):
        self.custom_local_jaw_cp = tf_utils.posestamped2g(msg)

    def get_cp(self):
        while True:
            if self.custom_arm_cp is None:
                continue
            else:
                return self.custom_arm_cp

    def get_local_cp(self):
        while True:
            if self.custom_local_arm_cp is None:
                continue
            else:
                return self.custom_local_arm_cp

    def get_jaw_cp(self):
        while True:
            if self.custom_jaw_cp is None:
                continue
            else:
                return self.custom_jaw_cp

    def get_local_jaw_cp(self):
        while True:
            if self.custom_local_jaw_cp is None:
                continue
            else:
                return self.custom_local_jaw_cp
            
    def run(self):
        self.init_ik()
        self.init_custom_cp_subscribers()
        self.test()


def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    # app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    # args = parser.parse_args(app_args) 

    # example_name = type(crtk_servo_jp_example).__name__
    # ral = crtk.ral(example_name, args.namespace)
    # example = crtk_servo_jp_example(ral)
    # ral = crtk.ral("dvrk_ctrl")

    control_app = DVRK_CTRL("PSM1", 0.01)

    control_app.ral.spin_and_execute(control_app.run)


if __name__ == '__main__':
    main()