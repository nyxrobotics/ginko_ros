cmake_minimum_required(VERSION 2.8.3)
# Package name: ginko_motion_player
project(ginko_motion_player)

add_compile_options(-std=c++11)

# Finding Dependent CMake Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nodelet
  pluginlib

  decision_making_parser
  decision_making
  random_numbers
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nodelet pluginlib
)


###########
## FSM Build ##
###########
add_executable(fsm_ginko
 src/fsm/ginko_fsm.cpp
  src/ginko_timer.cpp
    src/ginko_player.cpp
)

add_executable(fsm_ginko_events src/fsm/ginko_fsm_events.cpp)
target_link_libraries(fsm_ginko ${catkin_LIBRARIES})
target_link_libraries(fsm_ginko_events ${catkin_LIBRARIES})
decision_making_parsing(src/fsm/ginko_fsm.cpp)
###########
## Build ##
###########

include_directories(
  include
  include/motions
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# add_executable(${PROJECT_NAME} src/fsm/ginko_motion.cpp)
# add_dependencies(${PROJECT_NAME}
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(${PROJECT_NAME} 
#   ${catkin_LIBRARIES} 
# )


#############
## Install ##
#############

# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
