#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <iostream>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0){
		return -1;
	}

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZ> segmentation;
	segmentation.setInputCloud(cloud);
	// Configure the object to look for a plane.
	segmentation.setModelType(pcl::SACMODEL_CYLINDER);
	// Use RANSAC method.
	segmentation.setMethodType(pcl::SAC_RANSAC);
	// Set the maximum allowed distance to the model.
	segmentation.setDistanceThreshold(0.01);
	// Enable model coefficient refinement (optional).
	segmentation.setOptimizeCoefficients(true);
	// Set minimum and maximum radii of the cylinder.
	segmentation.setRadiusLimits(0, 0.1);

	pcl::PointIndices inlierIndices;
	segmentation.segment(inlierIndices, *coefficients);

	if (inlierIndices.indices.size() == 0){
		PCL_ERROR ("No se pudo identificar un cilindro en base al modelo.");
	}else{
		pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);	
	} 
	pcl::io::savePCDFileASCII(argv[2], *inlierPoints);
}