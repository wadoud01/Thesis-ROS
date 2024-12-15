cmake_minimum_required(VERSION 3.1.3)
project(panda_moveit_config)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Add the sender executable
add_executable(pos_udp_sender src/pos_udp_sender.cpp)
target_link_libraries(pos_udp_sender ${catkin_LIBRARIES})

# Add the receiver executable
add_executable(upd_joint_pos_control src/upd_joint_pos_control.cpp)
target_link_libraries(upd_joint_pos_control ${catkin_LIBRARIES})

