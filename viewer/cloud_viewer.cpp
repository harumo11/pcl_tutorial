#include <iostream>
#include <memory>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

void viewer_one_off(pcl::visualization::PCLVisualizer& viewer){
}

void viewer_psycho(pcl::visualization::PCLVisualizer& viewer){
}

int main(int argc, char const* argv[])
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	return 0;
}
