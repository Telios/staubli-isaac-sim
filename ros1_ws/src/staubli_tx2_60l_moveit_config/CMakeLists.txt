cmake_minimum_required(VERSION 3.1.3)
project(staubli_tx2_60l_moveit_config)

find_package(catkin REQUIRED
  std_msgs
  message_generation
)

add_message_files(
  FILES
  BBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/move_group_python_interface_tutorial.py
  scripts/staubli_ik.py
  scripts/color_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
