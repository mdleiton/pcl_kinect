#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/segment_differences.h>

int main (int argc, char** argv){
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud_a) != 0){
    return -1;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *cloud_b) != 0){
    return -1;
  }
  
  pcl::SegmentDifferences<pcl::PointXYZRGB> difference;
  difference.setInputCloud(cloud_a);
  difference.setTargetCloud(cloud_b);
  difference.segment(*result);    

  
  // remover nan
  //std::vector<int> mapping;
  //pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);

  

  pcl::io::savePCDFileASCII (argv[3], *result);
  return (0);
}
