add_executable(test_vo_match test_vo_match.cpp)
target_link_libraries(test_vo_match libvo libconfig libbase libstereo libfeature)

add_executable(test_motion test_motion.cpp)
target_link_libraries(test_motion libvo libconfig libbase libstereo libfeature)
