include_directories(${tracking_2D_SOURCE_DIR}/include)
add_executable(tracking_2D main.cc tracking.cc kalman_filter.cc)
target_link_libraries(tracking_2D Eigen3::Eigen)