# Installation Guide


### 1. Install ROS 2 Humble

Follow the official instructions for installing ROS 2 Humble:

[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### 2. Install dVRK Packages for ROS 2

Clone and install the required dVRK ROS 2 packages (cisst-saw, crtk, dvrk) from their official repositories or sources and make sure the 1394 FireWire connection to the da Vinci dVRK works properly (Ubuntu 22.04 version):

[dVRK ROS2 Installation Guide](https://dvrk.readthedocs.io/en/latest/pages/software/compilation/ros2.html)

### 3. Ensure `​sitl_ros2_interfaces` Package is Installed

Make sure the  package [sitl_ros2_interfaces](https://github.com/hossein-haeri/sitl_ros2_interfaces) is installed in your ROS2 workspace src directory. If not, clone and build it within your workspace. This package is required for custom ros2 messages.

### 4. Install Python Packages

1. You will need `​pyquaternion` for quaternion operations. Install it using `​pip`:

`pip install pyquaternion`

2. Please install pyserial to access the arduino controlling the monopolar pedal.

`pip install pyserial`

### 5. Set Up Calibration Data

Ensure your calibration data is available in the `​~/aruco_data` directory. If this directory does not exist, create it:

`mkdir -p ~/aruco_data`

Populate this directory with your calibration data files.


## Additional Notes

- Ensure that all dependencies are installed and sourced properly.
- For any issues with package installation or building, refer to the corresponding documentation.
