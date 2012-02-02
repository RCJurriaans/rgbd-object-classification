
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

	  SegmentCloud(double thresholdin, double maxDistanceFilterin, double minDistanceFilterin){
		  setSegMethod(SegPlane);
		  threshold = thresholdin;
		  maxDistanceFilter = maxDistanceFilterin;
		  minDistanceFilter = minDistanceFilterin;
	  }

	  // Method of Segmentation
	  enum SegmentationMethod { SegBack, SegObj, SegPlane, SegNormHist };


	  // Get the points representing the smallest bounding box
	  pcl::ModelCoefficients getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, cv::Mat mask, cv::Rect ROI);
	  pcl::ModelCoefficients getSmallestBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr unorgCloud);

	  // Mask getting function, uses internally stored background
	  boost::shared_ptr<cv::Mat>
		  getMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
		  boost::shared_ptr<pcl::PointIndices> objectInliers,
		  pcl::PointIndices::Ptr planeInliers,
		  boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs,
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);

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
		  getPlaneMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<pcl::PointIndices> objectinliers,
			pcl::PointIndices::Ptr planeInliers, boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);

	  boost::shared_ptr<cv::Mat>
		  getPlaneMask(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {
		  boost::shared_ptr<pcl::PointIndices> objectinliers(new pcl::PointIndices);
		  pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
		  boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs(new pcl::ModelCoefficients);
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered( new pcl::PointCloud<pcl::PointXYZRGB>);
		  return getPlaneMask(input, objectinliers, planeInliers, planeCoeffs, cloud_filtered);
	  }

	  // Gets a Region Of Interest. Use mask as input if you have one already.
	  cv::Rect getROI(boost::shared_ptr<const cv::Mat> mask);

	  cv::Rect getROI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {return getROI(getMask(input));}

	  cv::Rect getROI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background)
	  {return getROI(getMask(input, background));}


	  // Returns a vector of cv::rects containing all regions of interest
	  boost::shared_ptr<std::vector<cv::Rect> > 
		  getROIS(boost::shared_ptr<const cv::Mat> mask);

	  boost::shared_ptr<std::vector<cv::Rect> >
		  getROIS(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input)
	  {return getROIS(getMask(input));}

	  boost::shared_ptr<std::vector<cv::Rect> >
		  getROIS(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr background)
	  {return getROIS(getMask(input, background));}


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


	  // get coefficients by supplying a ROI or a vector of ROIS
	  pcl::ModelCoefficients
		  getCoefficients(cv::Rect ROI, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

	  boost::shared_ptr<std::vector<pcl::ModelCoefficients> >
		  getCoefficients(boost::shared_ptr<std::vector<cv::Rect> > ROIS, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

		pcl::ModelCoefficients
			getCoefficients(cv::Rect ROI, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, boost::shared_ptr<const cv::Mat> mask);

	  // Gets an unorganised cloud, by copying only nonzero mask-points.
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		  getUnorgCloud(boost::shared_ptr<const cv::Mat> mask, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input);

	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
			SegmentCloud::getUnorgCloud(
				const cv::Mat mask,
				pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input,
				cv::Rect ROI);

	  //
	  boost::shared_ptr<cv::Mat> denoizeMask( boost::shared_ptr<cv::Mat> latestMask );

	  // 
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tijmenLikesHacking(
		boost::shared_ptr<cv::Mat> &maskOut,
		cv::Rect& ROIOut,
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
			
			boost::shared_ptr<pcl::PointIndices> objectInliers(new pcl::PointIndices);
			boost::shared_ptr<pcl::PointIndices> planeInliers(new pcl::PointIndices);
			boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs(new pcl::ModelCoefficients);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
			boost::shared_ptr<cv::Mat> fullMask = getMask(cloud, objectInliers, planeInliers, planeCoeffs, cloud_filtered);

			//boost::shared_ptr<std::vector<cv::Rect> > ROIs = getROIS(fullMask);
			//ROIOut = ROIs->at(0);
			ROIOut = getROI(fullMask);
			cv::Mat * masky = new cv::Mat((*fullMask)(ROIOut));
			maskOut.reset(masky);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr windowCloud = getWindowCloud(ROIOut, cloud);
			return this->getUnorgCloud(*maskOut, windowCloud, ROIOut);

			/*
			boost::shared_ptr<cv::Mat> fullMask = getMask(cloud);
			ROIOut = getROI(fullMask);
			cv::Mat * masky = new cv::Mat((*fullMask)(ROIOut));
			maskOut.reset(masky);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr windowCloud = this->getWindowCloud(ROIOut, cloud);
			return this->getUnorgCloud(maskOut, windowCloud);*/
	  }

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

	std::vector<boost::shared_ptr<cv::Mat> > maskHistory;

private:

};