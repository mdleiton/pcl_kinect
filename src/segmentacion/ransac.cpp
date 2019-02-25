#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0){
		return -1;
	}

	// RANSAC objects: model and algorithm.
	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
	// Set the maximum allowed distance to the model.
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();

	std::vector<int> inlierIndices;
	ransac.getInliers(inlierIndices);

	// Copy all inliers of the model to another cloud(inlierPoints ).
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inlierIndices, *inlierPoints);

	pcl::io::savePCDFileASCII(argb[2], *inlierPoints);
}

/*
pcl::SampleConsensusModelCircle2D: A 2D circle on the X-Y plane.
pcl::SampleConsensusModelCircle3D: A 3D circle (any plane).
pcl::SampleConsensusModelCone: A cone.
pcl::SampleConsensusModelCylinder: A cylinder.
pcl::SampleConsensusModelLine: A line.
pcl::SampleConsensusModelPlane: A plane.
pcl::SampleConsensusModelSphere: A sphere.
pcl::SampleConsensusModelStick: A stick (a line with user-given minimum/maximum width).
*/