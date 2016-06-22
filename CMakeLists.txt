cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.8 REQUIRED)
include_directories(/home/naman/Documents/Softwares/LeapSDK/include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(/home/naman/Documents/Softwares/LeapSDK/lib)
 
#add_compile_options(-Wall -Wextra -std=c++0x -pthread) # for all targets in current directory
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(${PCL_DEFINITIONS})

add_executable (main main.cpp)
target_link_libraries (main ${PCL_LIBRARIES} /home/naman/Documents/Softwares/LeapSDK/lib/x64/libLeap.so)
