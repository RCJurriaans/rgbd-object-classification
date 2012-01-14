
#ifndef RENDERER_H
#define RENDERER_H

#include "stdafx.h"
using namespace xn;
using namespace cv;



class Renderer
{
public:
	Renderer( boost::function<void (int)> inputCallback ) : 
		RGBImage(Mat(480, 640, CV_8UC3)),
		depthImage(Mat(480,640, CV_32FC1)),
		viewer(NULL),
		processInput(inputCallback),
		cloudViewerOpen(false)
		//cloudKeyboardCallback( cloudKeyboardCB )
	{
		//g_processInput = inputCallback;
	}

	void renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB);
	void renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth);
	void renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);
	void closeCloudDisplay();
	void openCloudDisplay();
	//void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie);

protected:
	pcl::visualization::CloudViewer* viewer;
	Mat RGBImage;
	Mat depthImage;
	boost::function<void (int)> processInput;
	bool cloudViewerOpen;
	//void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*) cloudKeyboardCallback;
private:
};

#endif