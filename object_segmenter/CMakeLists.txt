cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS "-g")

set(SUBSUBSYS_NAME object_segmenter)

# we need FindFLANN.cmake
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

project(object_segmenter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(segment_object src/main.cpp)
target_link_libraries(segment_object ${PCL_LIBRARIES})
