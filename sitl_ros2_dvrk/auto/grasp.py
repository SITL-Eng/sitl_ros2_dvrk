import crtk
from nodes.auto import grasp

import os

def main():
    home_dir = os.path.expanduser('~')

    params = {
        'node_name'         : 'grasp_gallb',
        'arm_name'          : 'PSM2',
        'expected_interval' : 0.01,
        'queue_size'        : 5,
        'bnd_topic'         : '/seg/bnd_3d',
        'ctrd_topic'        : '/seg/gallb/ctrd_3d',
        'skel_topic'        : '/seg/gallb/skel_3d',
        'key_topic'         : '/keyboard/grasp',
        'fbf_kpt_topic'     : '/kpt/fbf/psm_cp',
        'fbf_jaw_kpt_topic' : '/kpt/fbf/psmjaw_cp',
        'tf_path'           : os.path.join(home_dir, 'aruco_data/base_tfs.yaml'),
        'calib_fn'          : os.path.join(home_dir, 'aruco_data/psm2_calib_results_final_new_v2.mat'),
        'arm_ik_W'          : [2, 0.2],
        'align_dist'        : 0.01,
        'min_pull_dist'     : 0.002,
        'max_pull_dist'     : 0.006,
        'jaw_open_angle'    : 80,
        'jaw_close_angle'   : -15,
        'grasp_offset'      : 0.01,
        'window_size'       : 15,
        'window_thr'        : 3e-4,
        'Joffsets'          : [80, 80, 5, 100, 60, 90],
        'align_ratio'       : [0.7, 0.3],
    }

    ral = crtk.ral(params['node_name'])

    app = grasp.GRASP(ral, params)

    ral.spin_and_execute(app.run)

if __name__ == '__main__':
    main()