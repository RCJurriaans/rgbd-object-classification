

#include "stdafx.h"
#include "Renderer.h"

//using namespace xn;


void Renderer::renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniBGR) 
{
	oniBGR->fillRGB(640, 480, RGBImage.data);
	cv::cvtColor( RGBImage, RGBImage, cv::COLOR_BGR2RGB ); //CV_BGR2RGB );
	cv::imshow( "RGB Display", RGBImage);
}

void Renderer::renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage )
{
	cv::imshow( "RGB Display", *RGBImage);
}

void Renderer::renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth)
{
	oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
	cv::imshow("Depth Display", depthImage);
}

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	if(e.keyDown()) {
		(*processInput)(e.getKeyCode());
	}
}

void Renderer::renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)//without const, could do without copy
{
	/*
	if (!cloudViewerOpen) {
		if (viewer != NULL) {
			delete viewer;
			viewer = NULL;
		}
	}
	else {
		cout << "rendering cloud" << endl;
		if (viewer == NULL)
		{
			viewer = new pcl::visualization::CloudViewer("Cloud Display");
			viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		}
		viewer->showCloud (cloud);
	}*/
	//viewer->showCloud (cloud);
	//processInput( cvWaitKey(1) );

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud, *cloudCopy);
	
	mtx.lock();
	vizCloud = cloudCopy;
	mtx.unlock();
}

void Renderer::renderCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
	//pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZRGB>(*cloud, *cloudCopy);
	
	//mtx.lock();
	//vizCloud = cloudCopy;
	//mtx.unlock();
}


void Renderer::closeCloudDisplay()
{
	cloudViewerOpen = false; 
	//delete viewer;
	//viewer = NULL;
}

void Renderer::openCloudDisplay()
{
	cloudViewerOpen = true;
	//viewer = new pcl::visualization::CloudViewer("Cloud Display");
	//viewer->registerKeyboardCallback( &keyboardCB, &processInput );
}

// This method runs on the visualization thread
void Renderer::visCallback(pcl::visualization::PCLVisualizer& vis)
{
	if( !vizCloud ){
		boost::this_thread::sleep (boost::posix_time::milliseconds (1));
		return;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>());

	mtx.lock();
	  tempCloud.swap(vizCloud);
	mtx.unlock();

	if (!vis.updatePointCloud (tempCloud))
	{	
		vis.addPointCloud (tempCloud);
		vis.resetCameraViewpoint ();
	}
	vis.updatePointCloud(tempCloud);
}