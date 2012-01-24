

#include "stdafx.h"
#include "Renderer.h"

void Renderer::renderResults()
{

}

void Renderer::renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniBGR) 
{
	// Get the image opencv format
	oniBGR->fillRGB(640, 480, RGBImage.data);
	cv::cvtColor( RGBImage, RGBImage, cv::COLOR_BGR2RGB ); //CV_BGR2RGB );

	// Get the classification output
	results->mtx.lock();
		cv::Rect ROI;	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr c;
		boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > objects = results->getObjects();
		
		boost::shared_ptr<cv::Mat> mask = results->getMask();
		if (objects->size() > 0) {
			ROI = objects->at(0)->ROI;
			c = objects->at(0)->cloud;
		}
	results->mtx.unlock();

	cv::rectangle(RGBImage, ROI, cv::Scalar(255,0,0));
	cv::imshow( "RGB Display", RGBImage);

	if( mask ) {
		cv::imshow( "Mask display", *mask );
	}
}

void Renderer::renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage )
{
	cv::imshow( "RGB Display", *RGBImage);
}

void Renderer::renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth)
{
	oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
	//cv::rectangle(depthImage, ROI, cv::Scalar(100));
	cv::imshow("Depth Display", depthImage);
}