cmake_minimum_required(VERSION 3.5)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(viewer viewer.cpp)
add_executable(viewer_obj viewer_obj.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})
target_link_libraries(viewer_obj ${PCL_LIBRARIES})
