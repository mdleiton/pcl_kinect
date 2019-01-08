#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0){
		return -1;
	}
	// ConditionAnd , ConditionOr
	//disponibles: GT, GE, LT, LE, EQ.
	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZRGB>);
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.0))); //valores de puntos en el eje z mayores a cero  
	condition->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 1.0))); //valores de puntos en el eje z menores a 2.0 
	
	pcl::ConditionalRemoval<pcl::PointXYZRGB> filter;
	filter.setCondition(condition);
	filter.setInputCloud(cloud);
	filter.setKeepOrganized(true);  

	filter.filter(*filteredCloud); 	//si un punto no cumple la condición por defecto sus coordenadas XYZ se setean a nan nan nan.
	// estos valores no se visualizaran, pero se mantienen en el archivo ocupando espacio.	
	//filter.setUserFilterValue(0.0); // esto setea los valores que no cumplen la(s) condicion(es) a cero.

	// se deben remover los nan para eliminar dichos puntos del archivo.
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*filteredCloud, *filteredCloud, mapping);


	pcl::io::savePCDFileASCII(argv[2], *filteredCloud);
}