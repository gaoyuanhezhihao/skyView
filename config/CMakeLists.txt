set(Config_SRC
    Config.hpp
    Config.cpp
    str_proc.hpp
    str_proc.cpp)
add_library(libconfig STATIC ${Config_SRC})
target_include_directories(libconfig PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
