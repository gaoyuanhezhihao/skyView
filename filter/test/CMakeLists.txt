find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(line_filter_unittest line_filter_unittest.cpp)
target_link_libraries(line_filter_unittest filter gtest gtest_main gmock gmock_main Threads::Threads)
add_test(NAME line_filter_unittest COMMAND $<TARGET_FILE:line_filter_unittest>)
