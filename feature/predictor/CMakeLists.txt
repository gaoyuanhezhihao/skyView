add_library(predictor predictor.cpp predictor.hpp)
target_link_libraries(predictor base track feature ${OpenCV_LIBS})
target_include_directories(predictor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
