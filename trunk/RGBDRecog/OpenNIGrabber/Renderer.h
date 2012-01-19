
#ifndef RENDERER_H
#define RENDERER_H

#include "stdafx.h"
//using namespace xn;
//using namespace cv;

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie);

class Renderer
{
public:
	Renderer( boost::function<void (int)> inputCallback ) : 
		RGBImage(cv::Mat(480, 640, CV_8UC3)),
		depthImage(cv::Mat(480,640, CV_32FC1)),
		viewer(new pcl::visualization::CloudViewer("Cloud Viewer")),//NULL),
		processInput(inputCallback),
		cloudViewerOpen(false)
	{
		viewer->registerKeyboardCallback( &keyboardCB, &processInput );
	}

	void renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB);
	void renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage );
	void renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth);
	void renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	void renderCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
	void closeCloudDisplay();
	void openCloudDisplay();

protected:
	pcl::visualization::CloudViewer* viewer;
	cv::Mat RGBImage;
	cv::Mat depthImage;
	boost::function<void (int)> processInput;
	bool cloudViewerOpen;
private:
};

#endif