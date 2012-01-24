
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
		processInput(inputCallback)
	{
		cout <<"Renderer constructor" << endl;
	}

	void renderResults();

	// Old interface
	void renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB);
	void renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage );
	void renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth);
	
protected:
	cv::Mat RGBImage;
	cv::Mat depthImage;
	boost::function<void (int)> processInput;

	boost::mutex mtx;
		cv::Rect vizROI;

	boost::shared_ptr<ClassificationResults> results;
private:
};

#endif