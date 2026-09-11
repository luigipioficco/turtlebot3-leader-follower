# Base image: ROS2 Jazzy
FROM ros:jazzy

# RViz2 for 3D visualisation
RUN apt update -y && apt install -y ros-jazzy-rviz2

# ROS <-> Gazebo (Harmonic) interface. ros-gz-image provides the bridge for
# the camera images, which needs a dedicated node.
RUN apt update -y && apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-image

# cv_bridge, to convert Image messages into OpenCV images
RUN apt update -y && apt install -y ros-jazzy-cv-bridge

# tf2_geometry_msgs: registers the PointStamped/PoseStamped transforms
RUN apt update -y && apt install -y ros-jazzy-tf2-geometry-msgs

# OpenCV and numpy
RUN apt update -y && apt install -y python3-opencv python3-numpy

# pandas and matplotlib, used by scripts/analyze_results.py
RUN apt update -y && apt install -y python3-pandas python3-matplotlib

# handy for inspecting the camera stream while debugging
RUN apt update -y && apt install -y ros-jazzy-rqt-image-view

# NOTE: slam_toolbox, navigation2 and robot_localization are deliberately not
# installed: this project performs neither SLAM nor global planning.
