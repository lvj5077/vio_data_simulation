cmake_minimum_required(VERSION 2.8.3)
project(vio_data_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  rosbag
  cv_bridge
)


catkin_package(

)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/gener_alldata.cpp src/param.cpp src/utilities.cpp src/imu.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)

# add_executable(${PROJECT_NAME}_node_txt
#     src/gener_alldata_txt.cpp src/param.cpp src/utilities.cpp src/imu.cpp)
