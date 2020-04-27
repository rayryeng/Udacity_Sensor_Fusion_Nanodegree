cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project (tracking_1D)
 
find_package (Eigen3 REQUIRED NO_MODULE)
 
add_executable (tracking_1D tracking_1D.cc)
target_link_libraries (tracking_1D Eigen3::Eigen)