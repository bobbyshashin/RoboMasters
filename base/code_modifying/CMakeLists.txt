set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8)
project( base )
#find_package( OpenCV 2.4.11 EXACT REQUIRED )
find_package( OpenCV REQUIRED )
#find_package(Qt5 COMPONENTS Core Widgets Test Concurrent OpenGL REQUIRED)
#set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/qt5/lib/cmake/")
#find_package( CUDA QUIET REQUIRED)
#set(CUDA_NVCC_FLAGS
#    ${CUDA_NVCC_FLAGS};
#    -O3 -gencode arch=compute_22,code=sm_22)
add_executable( test base.cpp manifoldGPIO.cpp manifoldGPIO.h)
target_link_libraries( test ${OpenCV_LIBS} -lrt )
