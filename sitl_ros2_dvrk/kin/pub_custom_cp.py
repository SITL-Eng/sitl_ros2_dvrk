#!/usr/bin/env python3

import os

import rclpy

from nodes.kin import pub_custom_cp_node
        
def main(args=None):
    rclpy.init(args=args)
    params = {
        "node_name"     : "pub_custom_cp",
        "queue_size"    : 5,
        "cam_type"      : "30",
        "psm1_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm1_calib_results_final_new_v2.mat",
        "psm2_calib_fn" : "/home/" + os.getlogin() + "/aruco_data/psm2_calib_results_final_new_v2.mat",
        "ecm_calib_fn"  : "/home/" + os.getlogin() + "/aruco_data/ecm_calib_results_final_new_v2.mat"
    }
    node = pub_custom_cp_node.PUB_CUSTOM_DVRK_CP(params)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
