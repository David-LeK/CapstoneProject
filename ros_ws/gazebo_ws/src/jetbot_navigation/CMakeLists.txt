cmake_minimum_required(VERSION 3.0.2)
project(jetbot_navigation)

add_dependencies(jetbot_viz)

find_package(catkin REQUIRED COMPONENTS
  amcl
  jetbot_viz
  map_server
  move_base
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
