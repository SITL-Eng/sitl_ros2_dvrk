import dvrk

import sys
import numpy as np
np.seterr(all="ignore")
import math
        
def home_zero_position(arm, duration):
    arm.check_connections()
    if not arm.enable(10):
        sys.exit('failed to enable within 10 seconds')
    if not arm.home(10):
        sys.exit('failed to home within 10 seconds')
    # get current joints just to set size
    zero_jp = get_jp(arm)
    # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
    zero_jp.fill(0)
    if "PSM" in arm.name():
        arm.jaw.open(angle=math.radians(0)).wait()
    else:
        zero_jp[2] = 0.03
    run_arm_servo_jp(zero_jp, duration)

def home_init_position(arm, sleep_rate, expected_interval, init_jp, duration):
    # get current joints just to set size
    run_arm_servo_jp(arm, sleep_rate, expected_interval, init_jp, duration)

def home_test_position(arm, sleep_rate, expected_interval, duration):
    arm.check_connections()
    if not arm.enable(10):
        sys.exit('failed to enable within 10 seconds')
    if not arm.home(10):
        sys.exit('failed to home within 10 seconds')
    # get current joints just to set size
    zero_jp = get_jp(arm)
    # go to initial position
    zero_jp.fill(0)
    if "PSM" in arm.name():
        arm.jaw.open(angle=math.radians(0)).wait()
    zero_jp[2] = 0.12
    run_arm_servo_jp(arm, sleep_rate, expected_interval, zero_jp, duration)

# direct joint control example
def run_arm_servo_jp(arm, sleep_rate, expected_interval, goal, duration=5):
    initial_joint_position = get_jp(arm)
    samples = duration / expected_interval
    amplitude = (goal - initial_joint_position) / samples
    for i in range(int(samples)):
        cur_goal = initial_joint_position + i * amplitude
        arm.servo_jp(cur_goal, amplitude)
        sleep_rate.sleep()

def run_jaw_servo_jp(arm, sleep_rate, expected_interval, goal, duration=5):
    initial_joint_position = get_jaw_jp(arm)
    samples = duration / expected_interval
    amplitude = (goal - initial_joint_position) / samples
    for i in range(int(samples)):
        cur_goal = initial_joint_position + i * amplitude
        arm.jaw.servo_jp(cur_goal, amplitude)
        sleep_rate.sleep()

def run_full_servo_jp(arm, sleep_rate, expected_interval, arm_goal, jaw_goal=math.radians(0), duration=5):
    arm_init_jp = get_jp(arm)
    jaw_init_jp = get_jaw_jp(arm)
    samples = duration / expected_interval
    arm_amp = (arm_goal - arm_init_jp) / samples
    jaw_amp = (jaw_goal - jaw_init_jp) / samples
    for i in range(int(samples)):
        cur_arm_goal = arm_init_jp + i * arm_amp
        cur_jaw_goal = jaw_init_jp + i * jaw_amp
        arm.servo_jp(cur_arm_goal)
        arm.jaw.servo_jp(cur_jaw_goal)
        sleep_rate.sleep()

def get_jp(arm):
    while True:
        try:
            return np.copy(arm.setpoint_jp())
        except:
            continue

def get_jaw_jp(arm):
    while True:
        try:
            return np.copy(arm.jaw.setpoint_jp())
        except Exception as e:
            continue

# def get_cp(arm):
#     while True:
#         if custom_arm_cp is None:
#             continue
#         else:
#             return custom_arm_cp

# def get_local_cp(arm):
#     while True:
#         if custom_local_arm_cp is None:
#             continue
#         else:
#             return custom_local_arm_cp

# def get_jaw_cp(self):
#     while True:
#         if custom_jaw_cp is None:
#             continue
#         else:
#             return custom_jaw_cp

# def get_local_jaw_cp(self):
#     while True:
#         if custom_local_jaw_cp is None:
#             continue
#         else:
#             return custom_local_jaw_cp
