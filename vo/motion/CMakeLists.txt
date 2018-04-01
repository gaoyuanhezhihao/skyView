add_library(motion motion.cpp motion.hpp)
target_link_libraries(motion base match pose ${OpenCV_LIBS})
target_include_directories(motion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
