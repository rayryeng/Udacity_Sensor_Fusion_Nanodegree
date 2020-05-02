cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CXX_FLAGS "-Wall" "-pedantic")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(ukf_augmentation)

find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(ukf_augmentation main.cc ukf.cc)
target_link_libraries(ukf_augmentation Eigen3::Eigen)