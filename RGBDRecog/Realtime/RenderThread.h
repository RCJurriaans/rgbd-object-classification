
#pragma once

#include "stdafx.h"
/*

class RenderThread
{
public:
	
	RenderThread() :
	//  inputMutex(new boost::mutex()),
	  cloud(new pcl::PointCloud<pcl::PointXYZRGB>())
	  ,viewer()//new pcl::visualization::PCLVisualizer ("3D Viewer"))
	{
		cout << "RenderThread constructor" << endl;
	//	viewer->setBackgroundColor (0, 0, 0);
	//	////pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c( new pcl::PointCloud<pcl::PointXYZRGB>() );
	//	////viewer->addPointCloud<pcl::PointXYZRGB> (c);
	//	viewer->addPointCloud(cloud);
	//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	//	viewer->addCoordinateSystem (1.0);
	//	viewer->initCameraParameters ();
	}

	
	  void run();

	//template <class T>
	  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	

	//boost::shared_ptr<boost::mutex> inputMutex;
	boost::mutex inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

private:
	RenderThread( const RenderThread& other ); // non construction-copyable
    RenderThread& operator=( const RenderThread& ); // non copyable
};
*/