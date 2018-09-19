cmake_minimum_required(VERSION 2.8.3)
# Package name: ginko_joint_controller
project(ginko_joint_controller)

add_compile_options(-std=c++11)

# Finding Dependent CMake Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nodelet
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nodelet pluginlib
)

###########
## Build ##
###########




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_DEP 
  src/serial.cpp
)
add_dependencies(
  ${PROJECT_NAME}_DEP 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME} src/ginko_controller.cpp)
# add_dependencies(ginko_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ginko_controller ${catkin_LIBRARIES})


target_link_libraries(${PROJECT_NAME}_DEP
  ${catkin_LIBRARIES}
)

# add_library(${PROJECT_NAME}
#   src/futaba_driver.cpp
# )


# add_library(${PROJECT_NAME}
#   src/ginko_controller.cpp
# )

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_DEP
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${PROJECT_NAME}_DEP
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
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
