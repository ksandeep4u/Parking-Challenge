cmake_minimum_required(VERSION 3.9.0)
project(parking)

set (CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(park main.cpp)

target_link_libraries(park ${PCL_LIBRARIES} ${VTK_LIBRARIES})
