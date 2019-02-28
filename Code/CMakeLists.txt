cmake_minimum_required(VERSION 3.2)
project( mapper )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(mapper main.cpp Minimap.cpp)
target_link_libraries(mapper ${OpenCV_LIBS})

if (CMAKE_COMPILER_IS_GNUCC)
	set_property( TARGET mapper APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )
endif (CMAKE_COMPILER_IS_GNUCC)
