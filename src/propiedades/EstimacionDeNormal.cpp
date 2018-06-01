/*
the estimation method uses the nearest neighbors (the points that are closest to the one we are calculating the normal for) to find out the tangent plane and the normal vector. 
You can customize the method with the search radius (think about a sphere of that radius, centered in the point; 
all neighboring points that lie within will be used for the computation) and the viewpoint (by default, the output normals will be directionless; 
by supposing that all vectors must point towards the camera - because otherwise they would belong to surfaces that are not visible from the sensor 
Normals are also important because they give us information about the curvature of the surface at some point
*/

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv) {

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}

	pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	// A kd-tree is a data structure that makes searches efficient. The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal>(cloud, normals, 20, 0.03, "normals");
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
