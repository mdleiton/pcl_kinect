#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], *cloud) != 0){
		return -1;
	}
	// ConditionAnd , ConditionOr
	//disponibles: GT, GE, LT, LE, EQ.
	pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZRGBNormal>);
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGBNormal>("z", pcl::ComparisonOps::GT, 0.0))); //valores de puntos en el eje z mayores a cero  
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGBNormal>("z", pcl::ComparisonOps::LT, 2.0))); //valores de puntos en el eje z menores a 2.0 
	
	pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> filter;
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	//si un punto no cumple la condición será removido.
	filter.setKeepOrganized(true);
	filter.setUserFilterValue(0.0);

	filter.filter(*filteredCloud);
	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
}