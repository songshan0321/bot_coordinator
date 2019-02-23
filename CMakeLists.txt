cmake_minimum_required(VERSION 2.8.3)
project(esc_bot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  move_base_msgs
  actionlib
)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES action_goal_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs move_base_msgs actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# One per executable!
#add_executable(esc_bot src/action_goal_cpp.cpp)
#target_link_libraries(esc_bot ${catkin_LIBRARIES})
#add_dependencies(esc_bot ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(DIRECTORY param
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY maps
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY rviz
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3")