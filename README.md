# NVIDIA Isaac Sim for Staubli TX2-60L

This repository contains the necessary files to run the NVIDIA Isaac Sim with the Staubli TX2-60L robot with the ROS1 or ROS2 bridge. Furthermore, it is also possible to directly control the robot with ROS1 using moveit.


## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Further instructions](#further-instructions)
  - [ROS1](ros1_ws/README.md)
  - [ROS2](ros2_ws/README.md)

## Prerequisites
It is highly recommended to use a computer with a dedicated NVIDIA RTX GPU. Isaac Sim requires a powerful GPU to run smoothly. This repository was tested on a computer with an NVIDIA RTX 4090 with Ubuntu 22.04.

Follow the installation guide for the NVIDIA Isaac Sim [here](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html). It is recommended to install the NVIDIA driver 525 or later.

Install Docker from the official website [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository). This repository was tested with the Docker version 25.0.3 and Docker Compose version v2.24.6. Also make sure to follow the post-installation steps for Docker [here](https://docs.docker.com/engine/install/linux-postinstall/).

Install the NVIDIA Container Toolkit from the official website [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt).

Install tmux and tmuxp for easier management of the Isaac Sim and ROS1/ROS2 terminals. Install tmux/tmuxp with the following command:
```bash
sudo apt install tmux tmuxp
```
## Installation
Clone the repository with the following command:
```bash
git clone https://gitlab.tuwien.ac.at/e193-03-virtual-and-augmented-reality/staubli/staubli_isaac_sim.git
```
For ease of use create the following aliases in your .bashrc file or equivalent bash configuration file:
```bash
alias ds='docker stop $(docker ps -a -q)'
alias kt='tmux kill-session'
alias ka='ds && kt'
```
If you are inside a tmuxp session and want to kill all the running containers and the tmux session, you can use the following command:
```bash
ka
```
If you don't want to kill the tmux session but allow the containers to run in the background, you can use the following command:
```bash
kt
```
## Further instructions
See the README.md file in the ros1_ws/ and ros2_ws/ for further instructions on how to build/run Isaac Sim with the Staubli TX2-60L robot.
### [ROS1](ros1_ws/)
### [ROS2](ros2_ws/)

