cmake_minimum_required(VERSION 3.9.0)
project(parking)

set (CMAKE_CXX_STANDARD 11)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB parking_SRC
    "*.h"
    "*.cpp"
)

add_executable(park ${parking_SRC})


target_link_libraries(park ${PCL_LIBRARIES} ${VTK_LIBRARIES})
