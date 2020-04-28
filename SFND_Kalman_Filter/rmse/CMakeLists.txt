cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(rmse)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(rmse main.cc)
target_link_libraries(rmse Eigen3::Eigen)


