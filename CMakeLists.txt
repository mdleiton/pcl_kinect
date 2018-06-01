cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# entrada/salida
add_executable (pcd_lectura ./src/io/lectura_nube_puntos.cpp)
target_link_libraries (pcd_lectura ${PCL_LIBRARIES})

