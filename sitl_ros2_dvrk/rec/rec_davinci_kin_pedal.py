#! /usr/bin/env python3

# general libraries
import os

# ros2 libraries
import rclpy

# nodes
from nodes.rec import rec_davinci_kin_pedal_node

def main(args=None):
    rclpy.init(args=args)

    params = {
        "node_name"       : "rec_dvrk_kin",
        "queue_size"      : 5,
        "slop"            : 0.015,
        "save_path"       : "/home/" + os.getlogin() + "/dvrk_rec/trial_8",
    }
    
    topic_names = [
        "/ECM/custom/setpoint_cp",
        "/ECM/custom/local/setpoint_cp",
        "/ECM/measured_js",
        "/MTML/gripper/measured_js",
        "/MTML/local/measured_cp",
        "/MTML/measured_js",
        "/MTML/measured_cp",
        "/MTMR/gripper/measured_js",
        "/MTMR/local/measured_cp",
        "/MTMR/measured_js",
        "/MTMR/measured_cp",
        "/PSM1/custom/setpoint_cp",
        "/PSM1/custom/local/setpoint_cp",
        "/PSM1/jaw/measured_js",
        "/PSM1/measured_js",
        "/PSM2/custom/setpoint_cp",
        "/PSM2/custom/local/setpoint_cp",
        "/PSM2/jaw/measured_js",
        "/PSM2/measured_js",
        "/pedal/clutch",
        "/pedal/camera",
        "/pedal/monopolar/read",
    ]

    app = rec_davinci_kin_pedal_node.REC_DVRK_KIN(params, topic_names)

    try:
        rclpy.spin(app)
    except Exception as e:
        print(e)
        pass
    finally:
        app.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
