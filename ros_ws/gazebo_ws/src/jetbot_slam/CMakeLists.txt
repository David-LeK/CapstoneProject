cmake_minimum_required(VERSION 3.0.2)
project(jetbot_slam)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)