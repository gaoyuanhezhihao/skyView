add_library(filter line_cross_filter.cpp line_cross_filter.hpp)
target_link_libraries(filter config base)
target_include_directories(filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(test)
add_subdirectory(sobel)

INSTALL(TARGETS filter ARCHIVE
        DESTINATION lib)
