cmake_minimum_required(VERSION 3.5)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_visualizer pcl_visualizer.cpp)
add_executable(pcl_visualizer_obj pcl_visualizer_obj.cpp)
add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(pcl_visualizer ${PCL_LIBRARIES})
target_link_libraries(pcl_visualizer_obj ${PCL_LIBRARIES})
target_link_libraries(cloud_viewer ${PCL_LIBRARIES})
