#include "stdafx.h"
#include "SegmentCloud.h"

#include <stdio.h>
#include <string>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <limits>
#include <ctime>
#include <math.h>

// Get Different Cloud types
void SegmentCloud::getNaNCloud()
{
	int nmax;
	BooleanMask->release();
	BooleanMask = new cv::Mat( inputCloud->height, inputCloud->width, CV_8UC1);
	
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
				BooleanMask->data[j*BooleanMask->step[0]+i*BooleanMask->step[1]] = 0;
			}
			else {
				// std::cout << "Width: " << i << ", Height: " << j << std::endl;

				BooleanMask->data[j*BooleanMask->step[0]+i*BooleanMask->step[1]] = 255;
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

}

// Get different masks
void SegmentCloud::getROI()
{

}



// Choose Segmentation method
SegmentCloud::SegmentationMethod SegmentCloud::getSegMethod(){
	return crtMethod;
}
void SegmentCloud::setSegMethod(SegmentCloud::SegmentationMethod method){
	crtMethod = method;
}

// Set necessary clouds
void SegmentCloud::setBackgroundImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	backgroundCloud = cloud;
}

void SegmentCloud::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	inputCloud = cloud;
	
}

void SegmentCloud::setThreshold(double thres)
{
	threshold = thres;
}

void SegmentCloud::setMaxDistanceFilter(double distFilt)
{
	maxDistanceFilter = distFilt;
}

void SegmentCloud::setMinDistanceFilter(double distFilt)
{
	minDistanceFilter = distFilt;
}
