#include <iostream>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

void show_help(char* program_name){
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << "cloud_file_name.[pcd|ply]" << std::endl;
	std::cout << "-h: Show this help" << std::endl;
}

int main(int argc, char** argv)
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		std::cout << "HELP will be shown" << std::endl;
		show_help(argv[0]);
		return 0;
	}

	//Fetch point cloud file from args 
	std::vector<int> file_names;

	file_names = pcl::console::parse_file_extension_argument(argc, argv, ".obj");

	if (file_names.size() != 1) {
		std::cout << "Load obj file" << std::endl;
		file_names = pcl::console::parse_file_extension_argument(argc, argv, ".obj");

		if (file_names.size() == 0) {
			std::cout << "Error No file [obj] was found" << std::endl;
			show_help(argv[0]);
			return -1;
		}
	}

	//Load file
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_prt(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PolygonMesh::Ptr mash(new pcl::PolygonMesh());
	if (pcl::io::loadOBJFile(argv[file_names[0]], *source_cloud_prt) < 0) {
		std::cout << "error can not load obj file" << std::endl;
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer("point cloud viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> viewer_color_hander(source_cloud_prt, 255, 255, 255);
	viewer.addPointCloud(source_cloud_prt, viewer_color_hander, "point_cloud");
	viewer.addCoordinateSystem();
	
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}


	return 0;
}

