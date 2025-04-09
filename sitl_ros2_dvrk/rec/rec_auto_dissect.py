#! /usr/bin/env python3

# general libraries
import os

# ros2 libraries
import rclpy

# nodes
from nodes.rec import rec_auto_dissect

def main(args=None):
    rclpy.init(args=args)

    params = {
        "node_name"       : "rec_dvrk_kin",
        "queue_size"      : 5,
        "slop"            : 0.25,
        "save_path"       : "/home/" + os.getlogin() + "/dvrk_rec/auto_dissect/full_rosbag_02",
    }
    
    topic_names = [
        "/kpt/pch/psmjaw_cp",
        "/kpt/fbf/psmjaw_cp",
        "/PSM1/custom/jaw/setpoint_cp",
        "/PSM2/custom/jaw/setpoint_cp",
        "/seg/gallb/skel_3d",
        "/seg/bnd_3d",
        # "/pedal/monopolar/read",
        "/target"
    ]

    app = rec_auto_dissect.REC_AUTO_DISSECT(params, topic_names)

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
