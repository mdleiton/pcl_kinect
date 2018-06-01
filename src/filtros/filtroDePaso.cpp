#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::PassThrough<pcl::PointXYZRGBNormal> filter;
	filter.setInputCloud(cloud);
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0.0, 2.0); //range de 0.0-2.0
	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);

}
