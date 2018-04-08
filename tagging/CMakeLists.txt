file(GLOB SRC "*.cpp" "*.hpp")
add_executable(tagging ${SRC})
target_link_libraries(tagging base haoLib::config stereo)
target_include_directories(tagging PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
