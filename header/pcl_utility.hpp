#pragma once
#include <iostream>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//This function display help
void show_help(char* program_name) {
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h: Show this help." << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_file(int argc, char** argv){

	//Fetch point cloud file name in arguments | Works with PCD and PLY files
	std::vector<int> filenames_ply;
	std::vector<int> filenames_pcd;
	std::vector<int> filenames_obj;

	//Load file | Work with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

	filenames_ply = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
	filenames_pcd = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	filenames_obj = pcl::console::parse_file_extension_argument(argc, argv, ".obj");

	if (filenames_ply.size() == 1) {
		if (pcl::io::loadPLYFile(argv[filenames_ply[0]], *source_cloud_ptr) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames_ply[0]] << std::endl << std::endl;
			show_help(argv[0]);
			std::exit(-1);
		}
	}
	else if (filenames_pcd.size() == 1) {
		if (pcl::io::loadPCDFile(argv[filenames_pcd[0]], *source_cloud_ptr) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames_pcd[0]] << std::endl << std::endl;
			show_help(argv[0]);
			std::exit(-1);
		}
	}
	else if (filenames_obj.size() == 1) {
		if (pcl::io::loadOBJFile(argv[filenames_obj[0]], *source_cloud_ptr) < 0) {
			std::cout << "Error loading point cloud " << argv[filenames_obj[0]] << std::endl << std::endl;
			show_help(argv[0]);
			std::exit(-1);
		}
	}

	return source_cloud_ptr;
}

std::tuple<pcl::PointXYZ,				   //min_point_obb
	   	   pcl::PointXYZ, 				   //max_point_obb
	   	   pcl::PointXYZ, 				   //position
		   Eigen::Matrix3f,				   //rotation_matrix
		   Eigen::Vector3f,				   //major_vector
		   Eigen::Vector3f,				   //middle_vector
		   Eigen::Vector3f,				   //minor_vector
		   Eigen::Vector3f>				   //mass_center
		   get_obb(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_prt){

			   pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
			   feature_extractor.pcl::PCLBase<pcl::PointXYZ>::setInputCloud(source_cloud_prt);
			   feature_extractor.compute();

			   std::vector<float> moment_of_inertia;
			   std::vector<float> eccentricity;
			   pcl::PointXYZ min_point_OBB;
			   pcl::PointXYZ max_point_OBB;
			   pcl::PointXYZ position_OBB;
			   Eigen::Matrix3f rotational_matrix_OBB;
			   float major_value, middle_value, minor_value;
			   Eigen::Vector3f major_vector, middle_vector, minor_vector;
			   Eigen::Vector3f mass_center;

			   feature_extractor.getMomentOfInertia(moment_of_inertia);
			   feature_extractor.getEccentricity(eccentricity);
			   feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
			   feature_extractor.getEigenValues(major_value, middle_value, minor_value);
			   feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
			   feature_extractor.getMassCenter(mass_center);

			   return std::forward_as_tuple(min_point_OBB, 
					                        max_point_OBB, 
											position_OBB,
					                        rotational_matrix_OBB, 
									        major_vector,
											middle_vector,
											minor_vector,
									        mass_center);
}

//Add wireframe cube to visualizer
void add_wire_flame_cube_to_visualizer(pcl::visualization::PCLVisualizer& viewer, pcl::PointXYZ position_obb, Eigen::Matrix3f rotation_matrix_obb, pcl::PointXYZ max_point_obb, pcl::PointXYZ min_point_obb){
	//Add cube to viewer
	Eigen::Vector3f position(position_obb.x, position_obb.y, position_obb.z);
	Eigen::Quaternionf quat(rotation_matrix_obb);
	viewer.addCube(position, quat, max_point_obb.x-min_point_obb.x, max_point_obb.y-min_point_obb.y, max_point_obb.z-min_point_obb.z, "OBB");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.3, 0.3, "OBB");
	viewer.setRepresentationToWireframeForAllActors();
}

//Add axis
void add_axis_to_visualizer(pcl::visualization::PCLVisualizer& viewer, Eigen::Vector3f center_vector, 
		                                                               Eigen::Vector3f major_vector, 
																	   Eigen::Vector3f middle_vector, 
																	   Eigen::Vector3f minor_vector){
	pcl::PointXYZ center(center_vector(0), center_vector(1), center_vector(2));
	pcl::PointXYZ x_axis(major_vector(0) + center_vector(0), major_vector(1)+center_vector(1), major_vector(2)+center_vector(2));
	pcl::PointXYZ y_axis(middle_vector(0)+center_vector(0), middle_vector(1)+center_vector(1), middle_vector(2)+center_vector(2));
	pcl::PointXYZ z_axis(minor_vector(0)+center_vector(0), minor_vector(1)+center_vector(1), minor_vector(2)+center_vector(2));
	viewer.addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer.addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer.addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

}

