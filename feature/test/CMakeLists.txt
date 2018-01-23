add_executable(test_pt_track test_pt_track.cpp)
target_link_libraries(test_pt_track libconfig libbase libstereo libfeature)


find_package(GTest REQUIRED)
find_package(Threads REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(test_line test_line.cpp)

target_link_libraries(test_line libfeature gtest gtest_main gmock gmock_main Threads::Threads)
#add_test(AllTest test_line)
add_test(NAME test_line COMMAND $<TARGET_FILE:test_line>)
