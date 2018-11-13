cmake_minimum_required(VERSION 3.5)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(moment_of_inerita moment_of_inerita.cpp)
target_link_libraries(moment_of_inerita ${PCL_LIBRARIES})
