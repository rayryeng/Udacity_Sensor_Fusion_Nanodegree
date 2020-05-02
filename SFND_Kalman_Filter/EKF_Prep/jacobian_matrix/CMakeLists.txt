cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(jacobian_matrix)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(jacobian_matrix main.cc)
target_link_libraries(jacobian_matrix Eigen3::Eigen)