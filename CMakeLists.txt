cmake_minimum_required(VERSION 3.5)
project(ccpp_demo)

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

find_package(OpenCV 3 REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)

add_executable(ccpp_demo main.cpp)
target_link_libraries(ccpp_demo ${OpenCV_LIBS})