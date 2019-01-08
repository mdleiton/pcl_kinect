#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <iostream>

// If this function returns true, the candidate point will be added to the cluster of the seed point.
bool customCondition(const pcl::PointXYZRGB& seedPoint, const pcl::PointXYZRGB& candidatePoint, float squaredDistance){
	// modificar a conveniencia
	if (candidatePoint.y < seedPoint.y) return false;
	return true;
}

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::ConditionalEuclideanClustering<pcl::PointXYZRGB> clustering;
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
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