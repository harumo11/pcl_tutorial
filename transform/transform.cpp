#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <eigen3/Eigen/Eigen>

//This function display help
void show_help(char* program_name) {
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h: Show this help." << std::endl;
}

//This is the main fucntion
int main(int argc, char** argv)
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		show_help(argv[0]);
		return 0;
	}


	//Fetch point cloud file name in arguments | Works with PCD and PLY files
	std::vector<int> filenames;
	bool file_is_pcd = false;

	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

	if (filenames.size() != 1) {
		filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

		if (filenames.size() != 1) {
			show_help(argv[0]);
			return -1;
		}
		else {
			file_is_pcd = true;
		}
	}

	//Load file | Work with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	//auto source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	//auto はEigen使ってる関係で，遅延評価が入って，エラーが出る

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile(argv[filenames[0]], *source_cloud_ptr) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			show_help(argv[0]);
			return -1;
		}
	}
	else {
		if (pcl::io::loadPLYFile(argv[filenames[0]], *source_cloud_ptr) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			show_help(argv[0]);
			return -1;
		}
	}
	
	//Reminder transform matrix
	// |1 0 0 x |
	// |0 1 0 y |
	// |0 0 1 z |
	// |0 0 0 1 |
	
	//METHOD #1 This is a "manual" method. perfect to understand but error prone!
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	//Define a rotation matrix (see wikipedia of rotation matrix)
	double theta = M_PI/4; //The angle of rotation in radians
	transform_1(0,0) = std::cos(theta);
	transform_1(0,1) = -std::sin(theta);
	transform_1(1,0) = sin(theta);
	transform_1(1,1) = cos(theta);
	//         (row, column)
	
	//Define a transration of 2.5 meter on the x axis
	transform_1(0,3) = 2.5;

	//Print the transoformation
	std::cout << "Method #1 using a Matrix4f " << std::endl;
	std::cout << transform_1 <<  std::endl;

	//METHOD #2:Using a affine3f
	//This method is easier and less error prone
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	//Define a traslation of 2.5 meter on the x axis.
	transform_2.translation() << 2.5, 0.0, 0.0;

	//The same rotation matrix as before, theta radians around Z axis.
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

	//Print the transformation
	std::cout << std::endl << "Method #2: using an Affine3f" << std::endl;
	std::cout << transform_2.matrix() << std::endl;

	//Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_prt(new pcl::PointCloud<pcl::PointXYZ> ());
	//You can either apply transform_1 or transform_2; the are same
	pcl::transformPointCloud(*source_cloud_ptr, *transformed_cloud_prt, transform_2);

	//Visualization
	std::cout << "Point cloud colors : white = original point cloud" << std::endl
		      << "red = transformed point cloud " << std::endl;
	pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

	//Define R,G,B color for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud_ptr, 255, 255, 255);

	//We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud(source_cloud_ptr, source_cloud_color_handler, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_color_hander(transformed_cloud_prt, 230, 20, 20);
	viewer.addPointCloud(transformed_cloud_prt, transformed_color_hander, "transformed_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	//viewer.setPosition(800,400);
	
	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}

	return 0;
}
