from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sitl_ros2_dvrk'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.xml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sitl_ros2_interfaces'
        'cisst_msgs',
        'cisst_ros_bridge',
        'cisst_ros_crtk',
        'crtk_msgs',
        'crtk_python_client',
        'dvrk_arms_from_ros',
        'dvrk_camera_registration',
        'dvrk_config',
        'dvrk_hrsv_widget',
        'dvrk_model',
        'dvrk_python',
        'dvrk_robot',
        'dvrk_video',
    ],
    zip_safe=True,
    maintainer='sitleng',
    maintainer_email='sitldvrk@gmail.com',
    description="This package provides an interface for connecting to the dVRK console over ROS2, specifically modified for UIC SITL's da Vinci Robot.",
    license='MIT',
    entry_points={
        'console_scripts': [
            # custom kinematics
            "pub_custom_cp         = sitl_ros2_dvrk.kin.pub_custom_cp:main",
            "pub_custom_tf         = sitl_ros2_dvrk.kin.pub_custom_tf:main",
            "custom_control_test   = sitl_ros2_dvrk.kin.custom_control:main",
            # custom pedal
            'pub_pedal_dvrk        = sitl_ros2_dvrk.pedal.pub_pedal_dvrk:main',
            'pub_pedal_mp_r        = sitl_ros2_dvrk.pedal.pub_pedal_mp_r:main',
            'sub_pedal_mp_w        = sitl_ros2_dvrk.pedal.sub_pedal_mp_w:main',
            'pub_pedal_mp_w_test   = sitl_ros2_dvrk.pedal.pub_pedal_mp_w_test:main',
            'rec_davinci_kin_pedal = sitl_ros2_dvrk.rec.rec_davinci_kin_pedal:main',
            # auto dissection
            'grasp_key             = sitl_ros2_dvrk.auto.grasp_key:main',
            'grasp_gallb           = sitl_ros2_dvrk.auto.grasp:main',
            'dissect_key           = sitl_ros2_dvrk.auto.dissect_key:main',
            'dissect_gallb         = sitl_ros2_dvrk.auto.dissect:main',
            # recordings
            'rec_auto_dissect      = sitl_ros2_dvrk.rec.rec_auto_dissect:main',
        ],
    },
)

