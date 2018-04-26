add_executable(color_thres_demo demo.cpp)
add_executable(gray_thres gray_thres.cpp)
add_executable(thres_edge_test thres_edge_test.cpp)

target_link_libraries(thres_edge_test stereo feature edge_filter ${Boost_LIBRARIES} ${OpenCV_LIBS} haoLib::config base)
target_link_libraries(color_thres_demo ${Boost_LIBRARIES} ${OpenCV_LIBS} haoLib::config base)
target_link_libraries(gray_thres ${Boost_LIBRARIES} ${OpenCV_LIBS} haoLib::config base)
