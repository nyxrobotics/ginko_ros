cmake_minimum_required(VERSION 2.8.3)
# Package name: roboone_ring_tf_publisher
project(roboone_ring_tf_publisher)

#add_compile_options(-std=c++11)
add_compile_options(-std=c++17 -O3)

# Finding Dependent CMake Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nodelet
  tf2
  tf2_bullet
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_msgs
  tf2_py
  tf2_ros
  tf2_sensor_msgs
  tf2_tools
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

file(GLOB MY_SOURCES "src/*.cpp")
add_executable(${PROJECT_NAME} ${MY_SOURCES})

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
)



#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
