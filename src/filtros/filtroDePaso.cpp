#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.setFilterFieldName("z");
	filter.setFilterLimits(0.0, 2.0); //range de 0.0-2.0
	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);

}
