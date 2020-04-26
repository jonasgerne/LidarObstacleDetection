cmake_minimum_required(VERSION 3.10)
project(LidarObstacleDetection)

set(CMAKE_CXX_STANDARD 11)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.2 REQUIRED common io filters visualization)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(environment
        src/main.cpp
        src/processPointClouds.cpp
        src/render.cpp)
target_link_libraries (environment ${PCL_LIBRARIES})
