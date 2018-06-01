// http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

int main (int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
    return -1;
  }

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 
  pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
  
  seg.setOptimizeCoefficients (true); // Opcional

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0){
    PCL_ERROR ("No se pudo identificar un plano en base al modelo.");
    return (-1);
  }

  std::cerr << "Coeficientes del modelo: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " 
                                            << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  /*
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "<< cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;
  */
  return (0);
}
