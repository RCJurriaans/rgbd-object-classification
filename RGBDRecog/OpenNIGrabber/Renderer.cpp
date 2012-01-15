

#include "stdafx.h"
#include "Renderer.h"

using namespace xn;
using namespace cv;



void Renderer::renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniBGR) 
{
	oniBGR->fillRGB(640, 480, RGBImage.data);
	cvtColor( RGBImage, RGBImage, CV_BGR2RGB );
	imshow( "RGB Display", RGBImage);
	processInput( cvWaitKey(1) );
}

void Renderer::renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage )
{
	imshow( "RGB Display", *RGBImage);
	//processInput( cvWaitKey(1) );
}

void Renderer::renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth)
{
	oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
	imshow("Depth Display", depthImage);
	processInput( cvWaitKey(1) );
}

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	(*processInput)(e.getKeyCode());
}

void Renderer::renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	/*if (!cloudViewerOpen) {
		if (viewer != NULL) {
			delete viewer;
			viewer = NULL;
		}
	}
	else {
		if (viewer == NULL)
		{
			viewer = new pcl::visualization::CloudViewer("Cloud Display");
			viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		}
		viewer->showCloud (cloud);
	}*/
	viewer->showCloud (cloud);
	processInput( cvWaitKey(1) );
}
void Renderer::renderCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	viewer->showCloud (cloud);
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
