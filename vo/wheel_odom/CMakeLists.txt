add_library(wheel_odom wheel_odom.cpp wheel_odom.hpp)
target_link_libraries(wheel_odom haoLib::config motion BA)
target_include_directories(wheel_odom PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(test)
