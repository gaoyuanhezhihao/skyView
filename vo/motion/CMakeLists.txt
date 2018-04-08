add_library(motion motion.cpp motion.hpp frame_pose.cpp frame_pose.hpp)
target_link_libraries(motion base match pose ${OpenCV_LIBS})
target_include_directories(motion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
