cmake_minimum_required(VERSION 3.5)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(transform transform.cpp)
target_link_libraries(transform ${PCL_LIBRARIES})
