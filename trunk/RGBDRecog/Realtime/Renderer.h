
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
		cloudViewerOpen(true),
		vizCloud()//new pcl::PointCloud<pcl::PointXYZRGB>())
	{
		//viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		boost::function1<void, pcl::visualization::PCLVisualizer&> f = boost::bind(&Renderer::visCallback, this, _1);
		viewer->runOnVisualizationThread(f);
		
	}
	void visCallback(pcl::visualization::PCLVisualizer& vis);// {cout << "vizthread"<<endl;}

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

	boost::mutex mtx;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizCloud;
private:
};

#endif