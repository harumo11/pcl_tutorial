#include <iostream>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>

int main(void)
{
	
	//点群データ(x,y,z)を持つ
	pcl::PointCloud<pcl::PointXYZ> cloud;

	//Fill in the cloud data
	cloud.width = 5;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); i++) {
		cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}

	pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd. " << std::endl;

	for (size_t i = 0; i < cloud.points.size(); i++) {
		std::cerr << "\t" << cloud.points[i].x 
			      << "\t" << cloud.points[i].y
				  << "\t" << cloud.points[i].z << std::endl;
	}

	return 0;
}

