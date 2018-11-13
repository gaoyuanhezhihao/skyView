add_executable(floor_vo floor_odom.cpp)
target_link_libraries(floor_vo haoLib::config ${OpenCV_LIBS} base feature stereo vo match track motion predictor ${Boost_LIBRARIES} wheel_odom BA)
