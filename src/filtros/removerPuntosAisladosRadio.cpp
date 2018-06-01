/* The radius-based outlier removal is the simplest method of all. You must specify a search radius and the minimum number of neighbors
than a point must have to avoid being labelled as outlier. The algorithm will then iterate through all points 
(which can be slow in if the cloud is big) and perform the check: if less than that number of points are found within the radius,
it is removed. */
#include <pcl/io/pcd_io.h>
#include <pcl/filters/radius_outlier_removal.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> filter;
	filter.setInputCloud(cloud);
	// 
	filter.setRadiusSearch(0.15); //Every point must have 10 neighbors within 15cm, or it will be removed.
	filter.setMinNeighborsInRadius(10);

	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
}