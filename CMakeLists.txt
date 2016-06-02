cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)
include_directories(/Users/gautamgupta/Documents/LeapSDK/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(/Users/gautamgupta/Documents/LeapSDK/lib)

add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp ref.cpp)
target_link_libraries (main ${PCL_LIBRARIES} /Users/gautamgupta/Documents/LeapSDK/lib/libLeap.dylib)