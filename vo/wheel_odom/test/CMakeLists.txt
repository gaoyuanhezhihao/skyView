find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(wheel_odom_unittest wheel_odom_unittest.cpp)
target_link_libraries(wheel_odom_unittest wheel_odom gtest gtest_main gmock gmock_main Threads::Threads)
add_test(NAME wheel_odom_unittest COMMAND $<TARGET_FILE:wheel_odom_unittest>)

add_executable(test_wheel_odom test_wheel_odom.cpp)
target_link_libraries(test_wheel_odom haoLib::config ${OpenCV_LIBS} base feature stereo vo match track motion predictor wheel_odom)


add_executable(select_test_wheel_odom select_test_wheel_odom.cpp)
target_link_libraries(select_test_wheel_odom haoLib::config ${OpenCV_LIBS} base feature stereo vo match track motion predictor wheel_odom)
