cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_read)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# entrada/salida
add_executable (pcd_lectura ./src/io/lectura_nube_puntos.cpp)
target_link_libraries (pcd_lectura ${PCL_LIBRARIES})

#filtros
add_executable (removerNAN ./src/filtros/removerNAN.cpp)
target_link_libraries (removerNAN ${PCL_LIBRARIES})

add_executable (removerPARadio ./src/filtros/removerPuntosAisladosRadio.cpp)
target_link_libraries (removerPARadio ${PCL_LIBRARIES})

add_executable (removerPuntosCondicion ./src/filtros/FiltrosCondicionales.cpp)
target_link_libraries (removerPuntosCondicion ${PCL_LIBRARIES})

add_executable (filtroPaso ./src/filtros/filtroDePaso.cpp)
target_link_libraries (filtroPaso ${PCL_LIBRARIES})

#propiedades
add_executable (EstimacionNormales ./src/propiedades/EstimacionDeNormal.cpp)
target_link_libraries (EstimacionNormales ${PCL_LIBRARIES})

add_executable (ExtraerIndices ./src/propiedades/extraerIndices.cpp)
target_link_libraries (ExtraerIndices ${PCL_LIBRARIES})


#segmentacion
add_executable (SegmentacionPlanos ./src/segmentacion/PlaneModelSegmentation.cpp)
target_link_libraries (SegmentacionPlanos ${PCL_LIBRARIES})

add_executable (SegmentacionEuclidiana ./src/segmentacion/EuclideanSegmentation.cpp)
target_link_libraries (SegmentacionEuclidiana ${PCL_LIBRARIES})

add_executable (SegmentacionCondicionadaEuclidiana ./src/segmentacion/ConditionalEuclideanSegmentation.cpp)
target_link_libraries (SegmentacionCondicionadaEuclidiana ${PCL_LIBRARIES})

add_executable (reconstruccion ./src/reconstruccion.cpp)
target_link_libraries (reconstruccion ${PCL_LIBRARIES})
