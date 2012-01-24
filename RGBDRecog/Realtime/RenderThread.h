
#pragma once

#include "stdafx.h"
#include "ClassificationResults.h"


//void visCallback(pcl::visualization::PCLVisualizer& vis);
//boost::function1<void, pcl::visualization::PCLVisualizer&> f = boost::bind(&Renderer::visCallback, this, _1);
//viewer->runOnVisualizationThread(f);
//viewer->registerKeyboardCallback( &keyboardCB, &processInput );
//void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
//{
//	boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
//	if(e.keyDown()) {
//		(*processInput)(e.getKeyCode());
//	}
//}

class RenderThread
{
public:
	
	RenderThread( boost::shared_ptr<ClassificationResults> res ) :
	//  inputMutex(new boost::mutex()),
	  cloud()
	  ,viewer(NULL)//new pcl::visualization::PCLVisualizer ("3D Viewer"))
	  ,results(res)
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
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	pcl::visualization::CloudViewer* viewer;

	//boost::shared_ptr<boost::mutex> inputMutex;
	boost::mutex inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;

	boost::shared_ptr<ClassificationResults> results;

private:
	RenderThread( const RenderThread& other ); // non construction-copyable
    RenderThread& operator=( const RenderThread& ); // non copyable
};
