cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(ukf_predict_radar)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(ukf_predict_radar main.cc ukf.cc)
target_link_libraries(ukf_predict_radar Eigen3::Eigen)