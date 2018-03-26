add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate config base stereo)

add_executable(skyview_dumper skyview_dumper.cpp)
target_link_libraries(skyview_dumper config base stereo)

