
#pragma once

#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include "ImageCalculation.h"

class SegmentCloud
{
	// Cloud to segment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud;
	// Optional Background image (only necessary for background subtraction)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;
	


public:
	SegmentCloud() {
	setSegMethod(SegBack);
	setThreshold(0.05);
	setMaxDistanceFilter(2);
	setMinDistanceFilter(0.1);
	}

	ImageCalculation imcalc;

	// Method of Segmentation
	enum SegmentationMethod { SegBack, SegObj, SegPlane, SegNormHist };
	SegmentationMethod crtMethod;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NaNCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr WindowCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr UnorgCloud;

	// Masks
	cv::Mat BooleanMask;
	
	// Thresholds
	double threshold;
	double maxDistanceFilter;
	double minDistanceFilter;

	// Get Different Cloud types
	void getNaNCloud();
	void getWindowCloud();
	void getUnorgCloud();

	// Choose segmentation method
	SegmentationMethod getSegMethod();
	void setSegMethod(SegmentationMethod);

	// Get different masks
	void getROI();

	// Set necessary clouds
	void setBackgroundImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

	void setThreshold(double);
	void setMaxDistanceFilter(double);
	void setMinDistanceFilter(double);
protected:

private:

};