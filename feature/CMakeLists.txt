file(GLOB SRC "*.cpp")
add_library(feature STATIC ${SRC})
target_link_libraries(feature config base filter ${OpenCV_LIBS} track)
target_include_directories(feature PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(test)
add_subdirectory(match)
add_subdirectory(track)
add_subdirectory(predictor)
