find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )
add_library(pose STATIC pose.cpp pose.hpp)
target_link_libraries(pose ${CERES_LIBRARIES})
target_include_directories(pose PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(test)
