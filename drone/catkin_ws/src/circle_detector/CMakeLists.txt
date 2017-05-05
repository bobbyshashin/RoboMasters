cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project( circle_detector )

find_package( OpenCV REQUIRED )
find_package( CUDA QUIET REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_22,code=sm_22)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dji_flight_logic
#  CATKIN_DEPENDS dji_sdk geometry_msgs roscpp sensor_msgs std_msgs tf tf2
#  DEPENDS system_lib
)
set(CMAKE_MODULE_PATH /usr/local/lib/cmake/)


add_executable(circle_detector
    src/circle_detector.cpp)

target_link_libraries(circle_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})
