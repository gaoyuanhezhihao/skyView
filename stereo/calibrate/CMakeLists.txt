add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate haoLib::config base stereo)

add_executable(skyview_dumper skyview_dumper.cpp)
target_link_libraries(skyview_dumper haoLib::config base stereo)

add_executable(select_calibrate select_calibrate.cpp)
target_link_libraries(select_calibrate haoLib::config base stereo)
