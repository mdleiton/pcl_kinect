pcl_kinetic
===========


Actualizar submódulos
---------------------
submódulo kinect2 - Captura de imágenes y nubes de puntos con el dispositivo kinect2 (kinect one) usando la librería freenect2  

	cd pcl_kinect
	git submodule update --init --recursive


Compilar
--------

	mkdir build
    cd build
	cmake ../
    make


Ejecutar 
--------
	./../bin/<ejecutable> ../kinect2/datos_capturados/<nube>.pcd  ../<nube_resultado>.pcd
