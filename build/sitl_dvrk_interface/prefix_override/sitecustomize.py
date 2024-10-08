import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hossein/ros2_ws/src/sitl_dvrk_interface/install/sitl_dvrk_interface'
