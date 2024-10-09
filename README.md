# Poorman's Installation Guide

Follow these steps to set up your environment for Poorman's ROS 2 project.

## Installation Steps

### 1. Install ROS 2 Humble

Follow the official instructions for installing ROS 2 Humble:

[ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

### 2. Install dVRK Packages for ROS 2

Clone and install the required dVRK ROS 2 packages from their official repositories or sources.

### 3. Install `​pyquaternion`

You will need `​pyquaternion` for quaternion operations. Install it using `​pip`:

\`\`\`​bash
pip install pyquaternion
\`\`\`​

### 4. Set Up Calibration Data

Ensure your calibration data is available in the `​~/aruco_data` directory. If this directory does not exist, create it:

\`\`\`​bash
mkdir -p ~/aruco_data
\`\`\`​

Populate this directory with your calibration data files.

### 5. Ensure `​sitl_dvrk_ros2_interfaces` Package is Installed

Make sure the `​sitl_dvrk_ros2_interfaces` package is installed in your ROS 2 workspace. If not, clone and build it within your workspace.

## Additional Notes

- Ensure that all dependencies are installed and sourced properly.
- For any issues with package installation or building, refer to the corresponding documentation.
