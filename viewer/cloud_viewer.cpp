#include <iostream>
#include <memory>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;

void viewer_one_off(pcl::visualization::PCLVisualizer& viewer){
	viewer.setBackgroundColor(1.0, 0.5, 1.0);
	pcl::PointXYZ o;
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);
	std::cout << "i onry run once" << std::endl;
}

void viewer_psycho(pcl::visualization::PCLVisualizer& viewer){
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop" << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible data race condition here:
	user_data++;
}

int main(int argc, char const* argv[])
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::io::loadPCDFile("../../data/lamppost.pcd", *cloud);

	pcl::visualization::CloudViewer viewer("Cloud Viewer");

	//実際に点群が描画されるまでここでブロックします
	viewer.showCloud(cloud);

	//複雑な機能にアクセスするためにはコールバック関数を使用してください．
	//それを通して，CloudViewerの基になっているクラスのPCLVisualizerクラスにアクセスします．
	
	//この関数は一度だけ呼ばれます
	viewer.runOnVisualizationThreadOnce(viewer_one_off);

	//この関数は描画のたびに呼ばれます．
	viewer.runOnVisualizationThread(viewer_psycho);

	while (!viewer.wasStopped()) {
		//ここで何か処理ができます．
		//データの書き込み・読み込みの競合は自分でなんとかしてください
		user_data++;
	}


	return 0;
}
