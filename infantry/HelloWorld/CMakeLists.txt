set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8)
project( base )
find_package( OpenCV REQUIRED )
find_package( CUDA QUIET REQUIRED)
set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3 -gencode arch=compute_22,code=sm_22)
add_executable( test main.cpp)
target_link_libraries( test ${OpenCV_LIBS} -lrt )
