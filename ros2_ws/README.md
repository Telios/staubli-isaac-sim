# Stäubli TX2-60L ROS2 with Isaac Sim
Currently, it is only possible to control the robot in simulation with ROS2. The real robot control is not yet possible with ROS2 as the VAL3 driver is not yet available.
## Installation
Once the repository is cloned, navigate to the cloned directory into the ros2_ws/ and run the following command to build the Docker containers:
```bash
tmuxp load tmuxp_configs/build_docker.yml
```

## Usage
Once the Docker containers are built, you can start to control the robot in simulation. To control the robot in simulation, you will need to start the Isaac Sim with the Omniverse launcher and select the omni.isaac.ros2_bridge as the ROS Bridge Extension. Once the Isaac Sim is running, you can open the scene located in ```ros2_ws/isaac_sim/scenes/staubli_tx2_60l_vrlab.usd```.

After that you can start the moveit controller with the following command:
```bash
tmuxp load tmuxp_configs/moveit2.yml
```
This command will start the moveit2 controller and rviz. You can now control the robot with the moveit2 interface.

![Moveit2 controller](../images/staubli_rviz2_isaac_sim.gif)