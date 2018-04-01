add_library(match match.cpp match.hpp)
target_link_libraries(match base track ${OpenCV_LIBS})
target_include_directories(match PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
