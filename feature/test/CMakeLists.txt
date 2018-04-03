add_executable(test_pt_track test_pt_track.cpp)
target_link_libraries(test_pt_track config base stereo feature)

add_executable(test_range_hough test_range_hough.cpp)
target_link_libraries(test_range_hough config base stereo feature)

add_executable(test_range_hough_track test_range_hough_track.cpp)
target_link_libraries(test_range_hough_track config base stereo feature)

add_executable(test_real_range_hough test_real_range_hough.cpp)
target_link_libraries(test_real_range_hough config base stereo feature)

add_executable(line_track line_track.cpp)
target_link_libraries(line_track config base stereo feature)

add_executable(edge_test edge_test.cpp)
target_link_libraries(edge_test config base stereo)

find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(line_unittest line_unittest.cpp)
target_link_libraries(line_unittest feature gtest gtest_main gmock gmock_main Threads::Threads)
add_test(NAME line_unittest COMMAND $<TARGET_FILE:line_unittest>)

add_executable(range_hough_unittest range_hough_unittest.cpp)
target_link_libraries(range_hough_unittest feature gtest gtest_main
    gmock gmock_main Threads::Threads)
add_test(NAME range_hough_unittest COMMAND $<TARGET_FILE:range_hough_unittest>)
