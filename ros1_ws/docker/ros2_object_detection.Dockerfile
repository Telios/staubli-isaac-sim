FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_bcf535ea3b9d16a854aaeb1701ab5a86
SHELL ["/bin/bash", "-c"]

# make ros2 workspaces
RUN mkdir -p ./ros2_ws/src

# clone the necessary repositories for ROS2
WORKDIR /ros2_ws/src
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
COPY external_src/isaac_ros_common /ros2_ws/src/isaac_ros_common
COPY external_src/isaac_ros_object_detection /ros2_ws/src/isaac_ros_object_detection

# build the ROS2 workspace
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# download dependencies for detectnet
RUN sudo apt update && sudo apt install ros-humble-isaac-ros-tensor-rt ros-humble-isaac-ros-triton ros-humble-isaac-ros-dnn-image-encoder -y

RUN /ros2_ws/src/isaac_ros_object_detection/isaac_ros_detectnet/scripts/setup_model.sh --height 640 --width 800 \
    --config-file /ros2_ws/src/isaac_ros_object_detection/isaac_ros_detectnet/resources/isaac_sim_config.pbtxt

#  Set entrypoint
RUN mkdir -p /usr/local/bin/scripts
COPY docker/entrypoints/detectnet_entrypoint.sh /usr/local/bin/scripts/detectnet_entrypoint.sh
COPY models/yolov8s.onnx /tmp/yolov8s.onnx
RUN chmod +x /usr/local/bin/scripts/detectnet_entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/scripts/detectnet_entrypoint.sh" ]
