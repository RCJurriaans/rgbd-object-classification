// PointCloudViewer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"





int _tmain(int argc, _TCHAR* argv[])
{
	// Initialize the point cloud for storing the depth of the frame:
	pcl::PointCloud<pcl::PointXYZ> cloud;
	//g_cloud = &cloud;
	cloud.width    = 640;
	cloud.height   = 480;
	cloud.is_dense = true;
	cloud.points.resize(cloud.width * cloud.height);



	return 0;
}

