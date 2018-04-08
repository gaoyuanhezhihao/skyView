add_executable(sobel_demo src/main.cpp)
target_link_libraries(sobel_demo ${Boost_LIBRARIES} ${OpenCV_LIBS} haoLib::config base)
