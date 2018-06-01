#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <iostream>

// If this function returns true, the candidate point will be added to the cluster of the seed point.
bool customCondition(const pcl::PointXYZRGBNormal& seedPoint, const pcl::PointXYZRGBNormal& candidatePoint, float squaredDistance){
	// modificar a conveniencia
	if (candidatePoint.y < seedPoint.y) return false;
	return true;
}

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> clustering;
	clustering.setClusterTolerance(0.09);
	clustering.setMinClusterSize(200);
	clustering.setMaxClusterSize(25000);
	clustering.setInputCloud(cloud);
	
	clustering.setConditionFunction(&customCondition);
	std::vector<pcl::PointIndices> clusters;
	clustering.segment(clusters);

	// itera a traves de cada punta a traves de cada cluster.
	int currentClusterNum = 1;
	for (std::vector<pcl::PointIndices>::const_iterator i = clusters.begin(); i != clusters.end(); ++i){
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
			cluster->points.push_back(cloud->points[*point]);
		cluster->width = cluster->points.size();
		cluster->height = 1;
		cluster->is_dense = true;
		if (cluster->points.size() <= 0) break;
		std::cout << "Cluster " << currentClusterNum << " tiene " << cluster->points.size() << " puntos." << std::endl;
		std::string fileName = "cluster" + boost::to_string(currentClusterNum) + ".pcd";
		pcl::io::savePCDFileASCII(fileName, *cluster);

		currentClusterNum++;
	}
}