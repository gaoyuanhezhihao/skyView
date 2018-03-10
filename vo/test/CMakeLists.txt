add_executable(test_vo_match test_vo_match.cpp)
target_link_libraries(test_vo_match vo config base stereo feature)

add_executable(test_motion test_motion.cpp)
target_link_libraries(test_motion vo config base stereo feature)

add_executable(test_vo_chain test_vo_chain.cpp) 
target_link_libraries(test_vo_chain vo config base stereo feature)

add_executable(vo_range_hough vo_range_hough.cpp)
target_link_libraries(vo_range_hough vo config base stereo feature)

add_executable(ceres_vo ceres_vo.cpp)
target_link_libraries(ceres_vo vo config base stereo feature)
