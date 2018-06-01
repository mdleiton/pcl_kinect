/* Euclidean segmentation is the simplest of all. It checks the distance between two points. If it is less than a threshold, 
both are considered to belong in the same cluster. It works like a flood fill algorithm: a point in the cloud is "marked" as "chosen" for the cluster. 
Then, it spreads like a virus to all other points that are near enough, and from those to even more points, until none new can be added. 
Then, a new cluster is initialized, and the procedure starts again with the remaining unmarked points. */
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	kdtree->setInputCloud(cloud);

	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> clustering;
	// Set cluster tolerance to 2cm (small values may cause objects to be divided in several clusters, whereas big values may join objects in a same cluster).
	clustering.setClusterTolerance(0.09);
	
	clustering.setMinClusterSize(200); // minima cantidad de puntos en un cluster
	clustering.setMaxClusterSize(25000); // máxima cantidad de puntos en un cluster
	clustering.setSearchMethod(kdtree);
	clustering.setInputCloud(cloud);
	std::vector<pcl::PointIndices> clusters;
	clustering.extract(clusters);

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