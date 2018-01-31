find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(ImgLogger_base_unittest ImgLogger_base_unittest.cpp)
target_link_libraries(ImgLogger_base_unittest base gtest gtest_main gmock gmock_main Threads::Threads)
add_test(NAME ImgLogger_base_unittest COMMAND $<TARGET_FILE:ImgLogger_base_unittest>)
