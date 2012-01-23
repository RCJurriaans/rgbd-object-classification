
#ifndef RENDERER_H
#define RENDERER_H

#include "stdafx.h"
#include "ClassificationResults.h"

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie);

class Renderer
{
public:
	Renderer( boost::function<void (int)> inputCallback,
			  boost::shared_ptr<ClassificationResults> res) : 
		results(res),
		RGBImage(cv::Mat(480, 640, CV_8UC3)),
		depthImage(cv::Mat(480,640, CV_32FC1)),
		//viewer(new pcl::visualization::CloudViewer("Cloud Viewer")),//NULL),
		processInput(inputCallback),
		cloudViewerOpen(true),
		vizCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
		//,testCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
	{
		cout <<"Renderer constructor" << endl;
		//viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		boost::function1<void, pcl::visualization::PCLVisualizer&> f = boost::bind(&Renderer::visCallback, this, _1);
//		viewer->runOnVisualizationThread(f);
		
		//hacks
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
		//cout << "loader: " << pcl::io::loadPCDFile<pcl::PointXYZRGB> ("img001.pcd", *testCloud) <<endl;
		//cout << testCloud <<endl;
		//testCloud = cloud_back;
	}
	void visCallback(pcl::visualization::PCLVisualizer& vis);// {cout << "vizthread"<<endl;}

	void renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB);
	void renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage );
	void renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth);
	void renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, cv::Rect ROI);
	void renderCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
	void closeCloudDisplay();
	void openCloudDisplay();
	

	void setResults( cv::Rect ROI );
protected:
	//pcl::visualization::CloudViewer* viewer;
	cv::Mat RGBImage;
	cv::Mat depthImage;
	boost::function<void (int)> processInput;
	bool cloudViewerOpen;

	boost::mutex mtx;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizCloud;
		cv::Rect vizROI;

	boost::shared_ptr<ClassificationResults> results;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr testCloud;
private:
};

#endif