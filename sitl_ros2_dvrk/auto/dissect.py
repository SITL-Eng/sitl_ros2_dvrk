import crtk
from nodes.auto import dissect

import os

def main():
    home_dir = os.path.expanduser('~')

    params = {
        'node_name'         : 'dissect',
        'arm_name'          : 'PSM1',
        'expected_interval' : 0.01,
        'queue_size'        : 5,
        'bnd_topic'         : '/seg/bnd_3d',
        'skel_topic'        : '/seg/gallb/skel_3d',
        'key_topic'         : '/keyboard/dissect',
        'pch_jaw_kpt_topic' : '/kpt/pch/psmjaw_cp',
        'tf_path'           : os.path.join(home_dir, 'aruco_data/base_tfs.yaml'),
        'calib_fn'          : os.path.join(home_dir, 'aruco_data/psm1_calib_results_final_new_v2.mat'),
        'arm_ik_W'          : [2.0, 0.2],
        'move_dist'         : 5e-4,
        'd_eps'             : 1e-3,
        'pedal_flag'        : True,
        'window_size'       : 15,
        'window_thr'        : 3e-4,
        'Joffsets'          : [80, 80, 5, 100, 80, 80],
    }

    ral = crtk.ral(params['node_name'])

    app = dissect.DISSECT(ral, params)

    ral.spin_and_execute(app.run)

if __name__ == '__main__':
    main()