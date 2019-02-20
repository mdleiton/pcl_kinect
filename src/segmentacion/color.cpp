#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <iostream>

int main(int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	kdtree->setInputCloud(cloud);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> clustering;
	clustering.setInputCloud(cloud);
	clustering.setSearchMethod(kdtree);

	// clusters smaller than this will be merged with their neighbors.
	clustering.setMinClusterSize(15);
	// Set the distance threshold, to know which points will be considered neighbors.
	clustering.setDistanceThreshold(25);
	// Color threshold for comparing the RGB color of two points.
	clustering.setPointColorThreshold(75);
	// Region color threshold for the postprocessing step: clusters with colors
	// within the threshold will be merged in one.
	clustering.setRegionColorThreshold(75);

	std::vector <pcl::PointIndices> clusters;
	clustering.extract(clusters);

	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;

		if (cluster->points.size() <= 1000)
			continue;
		std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}
