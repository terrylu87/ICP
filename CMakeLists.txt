cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(icp)

#find_package(OpenCV 4.1 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

#find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
 
# Executable for create matrix exercise
add_executable (icp src/icp.cpp src/main.cpp src/test.cpp)
#add_executable (icp src/main.cpp src/main.cpp src/test.cpp)
target_link_libraries (icp ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
