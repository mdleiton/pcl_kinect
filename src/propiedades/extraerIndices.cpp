#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudAll(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudExtracted(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloudAll) != 0){
		return -1;
	}
	//extraer indice de planos
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	
	pcl::SACSegmentation<pcl::PointXYZRGBNormal> segmentation;
	segmentation.setOptimizeCoefficients(true);
	segmentation.setModelType(pcl::SACMODEL_PLANE); //modelo del plano
	segmentation.setMethodType(pcl::SAC_RANSAC); 
	segmentation.setDistanceThreshold(0.01);
	segmentation.setInputCloud(cloudAll);

	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);

	segmentation.segment(*pointIndices, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	extract.setInputCloud(cloudAll);
	extract.setIndices(pointIndices);
	
	// We will extract the points that are NOT indexed (the ones that are not in a plane).
	extract.setNegative(true);
	extract.filter(*cloudExtracted);

	pcl::io::savePCDFileASCII(argv[2], *cloudExtracted);
}