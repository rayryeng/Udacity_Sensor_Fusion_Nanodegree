cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(generating_sigma_points)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(generating_sigma_points main.cc ukf.cc)
target_link_libraries(generating_sigma_points Eigen3::Eigen)