// pcdViewer.cpp : 
// A simple 

#include "stdafx.h"

#include <stdio.h>
#include <string>

#include <XnOS.h>
//#include <GL/glut.h>
#include <math.h>
#include <XnCppWrapper.h>

//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>

//#include <pcl/features/normal_3d.h>


int _tmain(int argc, _TCHAR* argv[])
{
	std::string path;
	std::cout << "Enter pcd file path" << std::endl;
	std::cin >> path;

	std::string type;
	std::cout << "Please enter cloud type: XYZRGB or XYZ" << std::endl;
	std::cin >> type;

	if (type == "XYZRGB")
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *cloud);

		pcl::visualization::CloudViewer viewer( "PCD Viewer");
		viewer.showCloud(cloud);

		while (!viewer.wasStopped ())
		{
			Sleep(1000);
		}
	}
	else if (type == "XYZ")
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::io::loadPCDFile<pcl::PointXYZ> (path, *cloud);

		pcl::visualization::CloudViewer viewer( "PCD Viewer");
		viewer.showCloud(cloud);

		while (!viewer.wasStopped ())
		{
			Sleep(1000);
		}
	}
	else
	{
		cout << "Invalid cloud type" << endl;
		cin.get();
	}

	return 0;
}

