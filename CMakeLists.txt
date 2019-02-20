cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# entrada/salida
add_executable (../bin/pcd_lectura ./src/io/lectura_nube_puntos.cpp)
target_link_libraries (../bin/pcd_lectura ${PCL_LIBRARIES})

#filtros
add_executable (../bin/removerNAN ./src/filtros/removerNAN.cpp)
target_link_libraries (../bin/removerNAN ${PCL_LIBRARIES})

add_executable (../bin/removerPARadio ./src/filtros/removerPuntosAisladosRadio.cpp)
target_link_libraries (../bin/removerPARadio ${PCL_LIBRARIES})

add_executable (../bin/removerPuntosCondicion ./src/filtros/FiltrosCondicionales.cpp)
target_link_libraries (../bin/removerPuntosCondicion ${PCL_LIBRARIES})

add_executable (../bin/filtroPaso ./src/filtros/filtroDePaso.cpp)
target_link_libraries (../bin/filtroPaso ${PCL_LIBRARIES})

#propiedades
add_executable (../bin/EstimacionNormales ./src/propiedades/EstimacionDeNormal.cpp)
target_link_libraries (../bin/EstimacionNormales ${PCL_LIBRARIES})

add_executable (../bin/ExtraerIndices ./src/propiedades/extraerIndices.cpp)
target_link_libraries (../bin/ExtraerIndices ${PCL_LIBRARIES})


#segmentacion
add_executable (../bin/SegmentacionPlanos ./src/segmentacion/PlaneModelSegmentation.cpp)
target_link_libraries (../bin/SegmentacionPlanos ${PCL_LIBRARIES})

add_executable (../bin/SegmentacionEuclidiana ./src/segmentacion/EuclideanSegmentation.cpp)
target_link_libraries (../bin/SegmentacionEuclidiana ${PCL_LIBRARIES})

add_executable (../bin/SegmentacionCondicionadaEuclidiana ./src/segmentacion/ConditionalEuclideanSegmentation.cpp)
target_link_libraries (../bin/SegmentacionCondicionadaEuclidiana ${PCL_LIBRARIES})

#add_executable (../bin/triangulation ./src/triangulation.cpp)
#target_link_libraries (../bin/triangulation ${PCL_LIBRARIES})

add_executable (../bin/diference ./src/segmentacion/diference.cpp)
target_link_libraries (../bin/diference ${PCL_LIBRARIES})

add_executable (../bin/color ./src/segmentacion/color.cpp)
target_link_libraries (../bin/color ${PCL_LIBRARIES})

#procesamiento
add_executable (../build/procesamiento ./src/procesamiento.cpp)
target_link_libraries (../build/procesamiento ${PCL_LIBRARIES})
