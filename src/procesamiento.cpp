// http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/conditional_removal.h>


int main (int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0){
    return -1;
  }

  // remover nan
  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

  //filtro condicional
  pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("x", pcl::ComparisonOps::LT, 1.1))); //valores de puntos en el eje z mayores a cero  
  condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 1.5))); //valores de puntos en el eje z menores a 2.0 
  
  pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
  filter.setCondition(condition);
  filter.setInputCloud(cloud);
  filter.setKeepOrganized(true);  
  filter.filter(*filteredCloud); 
  //nan
  pcl::removeNaNFromPointCloud(*filteredCloud, *filteredCloud, mapping);

  //filtro planos
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  
  seg.setOptimizeCoefficients (true); // Opcional

  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (filteredCloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0){
    PCL_ERROR ("No se pudo identificar un plano en base al modelo.");
    return (-1);
  }

  std::cerr << "Coeficientes del modelo: " << coefficients->values[0] << " " << coefficients->values[1] << " "<< coefficients->values[2] << " " 
                                            << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  


  pcl::PointCloud<pcl::PointXYZRGB> cloudPlane;

  for (size_t i = 0; i < inliers->indices.size (); ++i){
    pcl::PointXYZRGB point;
    point.x = cloud->points[inliers->indices[i]].x;
    point.y = cloud->points[inliers->indices[i]].y;
    point.z = cloud->points[inliers->indices[i]].z;
    point.rgb = cloud->points[inliers->indices[i]].rgb;
    cloudPlane.points.push_back(point);
  }
  cloudPlane.width = inliers-> indices.size ();
  cloudPlane.height =1;
  cloudPlane.is_dense = false;
  cloudPlane.points.resize (cloudPlane.width * cloudPlane.height);

  pcl::io::savePCDFileASCII (argv[2], cloudPlane);
  return (0);
}