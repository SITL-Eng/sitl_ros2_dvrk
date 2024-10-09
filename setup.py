from setuptools import find_packages, setup

package_name = 'sitl_dvrk_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sitl_dvrk_ros2_interfaces'
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
    maintainer='hossein',
    maintainer_email='haeri.hsn@gmail.com',
    description="This package provides an interface for connecting to the dVRK console over ROS2, specifically modified for UIC SITL's da Vinci Robot.",
    license='MIT',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pub_cp = sitl_dvrk_ros2.pub_cp:main",
            "pub_tf = sitl_dvrk_ros2.pub_tf:main"
        ],
    },
)

