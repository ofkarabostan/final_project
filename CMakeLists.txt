cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(CDD_client)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCD})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (
	CDD_client 
	CDD_client.cpp
	CDD.cpp
)
target_link_libraries (CDD_client ${PCL_LIBRARIES})
