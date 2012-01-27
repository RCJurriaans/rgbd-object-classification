
#pragma once

#include "stdafx.h"
#include "ImageCalculation.h"
#include <pcl/ModelCoefficients.h>

#include <boost/make_shared.hpp>

class SegmentCloud
{
public:

	SegmentCloud() :
	  crtMethod(SegBack),
		  threshold(0.05),
		  maxDistanceFilter(2),
		  minDistanceFilter(0.1)
	  {}

	  // Method of Segmentation
	  enum SegmentationMethod { SegBack, SegObj, SegPlane, SegNormHist };


	  // Mask getting function, uses internally stored background
	  boost::shared_ptr<cv::Mat>
		  getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<pcl::PointIndices> inliers);

	  // Overloaded functions
	 
	  boost::shared_ptr<cv::Mat>
		  getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);	

	  boost::shared_ptr<cv::Mat>
		  getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background);


	  // Main implementation of background-subtraction mask calculation
	  boost::shared_ptr<cv::Mat>
		  getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background, boost::shared_ptr<pcl::PointIndices> inliers);


	  // Main implementation of plane-subtraction mask calculation
	  boost::shared_ptr<cv::Mat>
		  getPlaneMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<pcl::PointIndices> objectinliers);
	  

	  // Gets a Region Of Interest. Use mask as input if you have one already.
	  cv::Rect getROI(boost::shared_ptr<const cv::Mat> mask);

	  cv::Rect getROI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {return getROI(getMask(input));}

	  cv::Rect getROI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background)
	  {return getROI(getMask(input, background));}


	  // Cuts out a rectangular, organised cloud from an input cloud
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getWindowCloud(const cv::Rect& ROI, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getWindowCloud(boost::shared_ptr<const cv::Mat> mask, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {return getWindowCloud( getROI(mask), input);}

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getWindowCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {return getWindowCloud( getROI(getMask(input)), input);}

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getWindowCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background)
	  {return getWindowCloud( getROI(getMask(input, background)), input);}


	  // Gets an unorganised cloud, by copying only nonzero mask-points.
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getUnorgCloud(boost::shared_ptr<const cv::Mat> mask, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

	  // Getters and setters
	  inline SegmentationMethod getSegMethod(){return crtMethod;}
	  inline void setSegMethod(SegmentationMethod method) {crtMethod = method;}
	  inline void setThreshold(double thres) {threshold = thres;}
	  inline void setMaxDistanceFilter(double distFilt) {maxDistanceFilter = distFilt;}
	  inline void setMinDistanceFilter(double distFilt) {minDistanceFilter = distFilt;}
	  inline void setBackground(pcl::PointCloud<pcl::PointXYZRGB>::Ptr bg) {background = bg;}


protected:

	ImageCalculation imcalc;

	// State
	double threshold;
	double maxDistanceFilter;
	double minDistanceFilter;
	SegmentationMethod crtMethod;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr background;

private:

};