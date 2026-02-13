FROM osrf/ros:humble-desktop-full
SHELL ["/bin/bash", "-c"]

# make ros2 workspaces
RUN mkdir -p ./ros2_ws/src

# clone the necessary repositories for ROS2
WORKDIR /ros2_ws/src
COPY src/ /ros2_ws/src/

# build the ROS2 workspace
WORKDIR /ros2_ws
RUN sudo rosdep update && apt update && rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && apt-get install -y build-essential ros-dev-tools gdb
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

#  Set entrypoint
RUN mkdir -p /usr/local/bin/scripts
COPY docker/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]
