cmake_minimum_required(VERSION 2.8.3)
project(aev)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  dbw_mkz_msgs
	autoware_build_flags
	message_filters
	automotive_navigation_msgs
	autoware_msgs
  can_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  dbw_mkz_msgs
  message_filters
  automotive_navigation_msgs
  autoware_msgs
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(aev_interface
  nodes/aev_interface/aev_interface_node.cpp
  nodes/aev_interface/aev_interface.cpp
)

add_executable(control_node
  nodes/control_node/control_node.cpp
)

target_link_libraries(aev_interface ${catkin_LIBRARIES})
target_link_libraries(control_node ${catkin_LIBRARIES})
add_dependencies(aev_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})

install(TARGETS aev_interface control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
