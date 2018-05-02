add_library(BA frame_map.cpp frame_map.hpp LandMarkMap.cpp LandMarkMap.hpp)
target_link_libraries(BA base)
target_include_directories(BA PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
