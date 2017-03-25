cmake_minimum_required(VERSION 2.8.3)
project(victoria_base)

find_package(catkin REQUIRED COMPONENTS victoria_nav_msgs victoria_sensor_msgs
  roslaunch sensor_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS victoria_nav_msgs victoria_sensor_msgs sensor_msgs
)

install(DIRECTORY config
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/victoria_base.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslaunch_add_file_check(launch)
