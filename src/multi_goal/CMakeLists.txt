cmake_minimum_required(VERSION 2.8.3)
project(multi_goal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  std_msgs
  move_base_msgs
  actionlib_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/getpoints.py
  src/goal_loop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
