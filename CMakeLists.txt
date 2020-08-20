cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_p2pl)

find_package(PCL 1.2 REQUIRED)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp_p2pl icp_p2pl.cpp)
target_link_libraries (icp_p2pl ${PCL_LIBRARIES})
