# syntax=docker/dockerfile:1.4
FROM osrf/ros:noetic-desktop-focal
SHELL ["/bin/bash", "-c"]

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    ros-noetic-catkin \
    tmux \
    git \
    python-is-python3 \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-moveit \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p ./catkin_ws/src
COPY --link ../src /catkin_ws/src/

WORKDIR /catkin_ws
RUN source /opt/ros/noetic/setup.bash && apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y
RUN source /opt/ros/noetic/setup.bash && catkin build 

# Update entrypoint to source setup.bash
RUN sed --in-place --expression \
      '$isource "/catkin_ws/devel/setup.bash"' \
      /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]