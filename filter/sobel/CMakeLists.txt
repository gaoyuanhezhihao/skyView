add_executable(runner src/main.cpp)
target_link_libraries(runner ${Boost_LIBRARIES} ${OpenCV_LIBS})
