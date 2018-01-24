set(Config_SRC
    Config.hpp
    Config.cpp
    str_proc.hpp
    str_proc.cpp)
add_library(config STATIC ${Config_SRC})
target_include_directories(config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
