add_library(track track.cpp track.hpp)
target_link_libraries(track base ${OpenCV_LIBS})
target_include_directories(track PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
