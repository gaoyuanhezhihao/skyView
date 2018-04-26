add_library(filter line_cross_filter.cpp line_cross_filter.hpp)
target_link_libraries(filter haoLib::config base)
target_include_directories(filter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(edge_filter edge_filter.cpp edge_filter.hpp)
target_link_libraries(edge_filter haoLib::config base)

add_subdirectory(test)
add_subdirectory(sobel)
add_subdirectory(color_thres)

INSTALL(TARGETS filter ARCHIVE
        DESTINATION lib)
