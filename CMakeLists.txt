cmake_minimum_required(VERSION 2.8.3)
project(line_err_estimator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( OpenCV 2 REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  rospy
  geometry_msgs
)

include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## add_executable(image_converter src/image_converter.cpp)
## target_link_libraries ( image_converter ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )
