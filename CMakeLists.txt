cmake_minimum_required(VERSION 2.8.3)
project(tag_detector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(OpenCV)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  set_blur_window_size.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node1 src/tag_detector_node1.cpp)
add_executable(${PROJECT_NAME}_node2 src/tag_detector_node2.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node1 ${catkin_LIBRARIES} 
                                       ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node2 ${catkin_LIBRARIES} 
                                       ${OpenCV_LIBRARIES})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node1
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
  target_link_libraries(${PROJECT_NAME}_node2
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


## Mark executables and/or libraries for installation
install(TARGETS tag_detector_node1 tag_detector_node2
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)