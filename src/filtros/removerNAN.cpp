#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

#include <iostream>
// parametros nube de puntos de entrada y de salida.
int main(int argc, char** argv){
	if (argc != 3){
		return -1;
	}
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){		
		return -1;
	}
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

	pcl::io::savePCDFileASCII(argv[2], *cloud);
}