cmake_minimum_required(VERSION 2.8.3)
project(student_proect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED )
##find_package(project_interface REQUIRED )

## Specify additional locations of header files
include_directories(
 include 
 ${OpenCV_INCLUDE_DIRS}
 ${project_interface_INCLUDE_DIRS}
)

## CREATE EXECUTABLE LIBRARY
add_library(student SHARED
   src/student_interface.cpp
   src/dubins.cpp 
   src/utils.cpp 
   src/clipper.cpp 
   src/clipper_addons.cpp 
   src/graph.cpp 
   src/open_edges.cpp 
   src/visgraph.cpp 
   src/graphPrint.cpp 
)

target_link_libraries(student
  ${OpenCV_LIBRARIES}
)

# Executable to test the implemented functions
add_executable(functions-testing src/main.cpp src/dubins.cpp src/utils.cpp src/clipper.cpp src/clipper_addons.cpp src/graph.cpp src/open_edges.cpp src/visgraph.cpp src/graphPrint.cpp)

target_link_libraries(functions-testing
  ${OpenCV_LIBRARIES}
)
