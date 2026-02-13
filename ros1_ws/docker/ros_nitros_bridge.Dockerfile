FROM nvcr.io/nvidia/isaac/ros:x86_64-ros2_humble_bcf535ea3b9d16a854aaeb1701ab5a86
SHELL ["/bin/bash", "-c"]

# setup ROS1 keys
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -


# install ROS1
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    ros-noetic-catkin \
    python-is-python3 \
    python3-pip \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# make ros1 and ros2 workspaces
RUN mkdir -p ./ros2_ws/src
RUN mkdir -p ./ros1_ws/src

# clone the necessary repositories for ROS2
WORKDIR /ros2_ws/src
RUN git clone https://github.com/ros2/ros1_bridge && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros.git
COPY external_src/isaac_ros_nitros_bridge /ros2_ws/src/isaac_ros_nitros_bridge
COPY external_src/isaac_ros_common /ros2_ws/src/isaac_ros_common

# clone the necessary repositories for ROS1
WORKDIR /ros1_ws/src
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_noetic_interfaces.git
COPY external_src/isaac_ros_nitros_bridge /ros1_ws/src/isaac_ros_nitros_bridge

# build the ROS1 workspace
WORKDIR /ros1_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make_isolated --install --ignore-pkg isaac_ros_nitros_bridge_ros2 -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs

# build the ROS2 workspace
WORKDIR /ros2_ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-skip ros1_bridge

# build the ros1_bridge after both workspaces are built, see https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nitros_bridge/index.html 
WORKDIR /ros1_ws
RUN source install_isolated/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/ && \ 
    colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure

#  Set entrypoint
RUN mkdir -p /usr/local/bin/scripts
COPY docker/entrypoints/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]
