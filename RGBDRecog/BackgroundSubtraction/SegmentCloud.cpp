#include "stdafx.h"
#include "SegmentCloud.h"

#include <stdio.h>
#include <string>

// PCL stuff
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Opencv stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <limits>
#include <ctime>
#include <math.h>

// Blobb stuff
#include "ImageAccess.h"



// Get Different Cloud types
void SegmentCloud::getNaNCloud()
{
	int nmax;
	BooleanMask.release();
	BooleanMask = cv::Mat::zeros( inputCloud->height, inputCloud->width, CV_8UC1);

	switch( getSegMethod() ){
	case SegBack:

		float point_backz;
		float point_imgz;

		nmax = inputCloud->width * inputCloud->height;
		for(int n=0; n < nmax ; n++ ){
			// Extract z value from both clouds
			point_backz = backgroundCloud->at(n).z;
			point_imgz  = inputCloud->at(n).z;

			// Recalculate indices in matrix from n
			int i = n % inputCloud->width;
			int j = ((n-i) / inputCloud->width);

			// Check whether the distance is smaller than a threshold
			// Check whether the distance is within the range of the RGB-D camera
			// Check for QNaN
			if(abs(point_backz-point_imgz)<threshold || point_imgz>maxDistanceFilter || point_imgz<minDistanceFilter || point_imgz != point_imgz){
				inputCloud->points[n].z = std::numeric_limits<float>::quiet_NaN();
				//BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 0;
			}
			else {
				BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 255;
			}
		}
		break;
	case SegObj:
		std::cout << "No such method yet" << std::endl;
		break;
	case SegPlane:
		std::cout << "No such method yet" << std::endl;
		break;
	case SegNormHist:
		std::cout << "No such method yet" << std::endl;
		break;
	}

};

void SegmentCloud::getWindowCloud()
{

}

void SegmentCloud::getUnorgCloud()
{
	int nmax;
	BooleanMask.release();
	BooleanMask = cv::Mat::ones( inputCloud->height, inputCloud->width, CV_8UC1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	switch( getSegMethod() ){
	case SegBack:

		float point_backz;
		float point_imgz;

		nmax = inputCloud->width * inputCloud->height;
		std::cout << "Created segment object" << std::endl;
		for(int n=0; n < nmax ; n++ ){
			// Extract z value from both clouds
			point_backz = backgroundCloud->at(n).z;
			point_imgz  = inputCloud->at(n).z;

			// Recalculate indices in matrix from n
			int i = n % inputCloud->width;
			int j = ((n-i) / inputCloud->width);

			// Check whether the distance is smaller than a threshold
			// Check whether the distance is within the range of the RGB-D camera
			// Check for QNaN
			if(abs(point_backz-point_imgz)<threshold || point_imgz>maxDistanceFilter || point_imgz<minDistanceFilter || point_imgz != point_imgz){
				BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 0;
			}
			else {
				segmentCloud->push_back(inputCloud->at(n));
				//BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 1;
			}
		}
		inputCloud->swap(*segmentCloud);

		break;
	case SegObj:
		std::cout << "No such method yet" << std::endl;
		break;
	case SegPlane:
		std::cout << "No such method yet" << std::endl;
		break;
	case SegNormHist:
		std::cout << "No such method yet" << std::endl;
		break;
	}

}

// Get mask
void SegmentCloud::getROI()
{
	IplImage ipl_bmask = BooleanMask;//cvCreateImage(cvSize(BooleanMask.size().height,BooleanMask.size().width),8,1);
	std::cout << "Created ipl version of mask " <<  std::endl;
	cvSetImageROI(&ipl_bmask, cvRect(0,0,ipl_bmask.width, ipl_bmask.height));
	//BwImage enter(ipl_bmask);
	imcalc.Calculate(&ipl_bmask, 1);
}


// Choose Segmentation method
SegmentCloud::SegmentationMethod SegmentCloud::getSegMethod(){
	return crtMethod;
}
void SegmentCloud::setSegMethod(SegmentCloud::SegmentationMethod method){
	crtMethod = method;
}

// Set background cloud
void SegmentCloud::setBackgroundImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	backgroundCloud = cloud;
}

// Set input cloud
void SegmentCloud::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	inputCloud = cloud;
}

// Set threshold for difference between background and input
void SegmentCloud::setThreshold(double thres)
{
	threshold = thres;
}

// Set max distance of points
void SegmentCloud::setMaxDistanceFilter(double distFilt)
{
	maxDistanceFilter = distFilt;
}

// Set min distance of points
void SegmentCloud::setMinDistanceFilter(double distFilt)
{
	minDistanceFilter = distFilt;
}
