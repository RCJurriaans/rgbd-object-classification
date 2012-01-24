
#pragma once

#include "stdafx.h"

struct FoundObject
{
	FoundObject(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
					cv::Rect r,
					int cl ) :
		cloud(c), ROI(r), classification(cl)
	{}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;	// Cloud of the segmented object
	cv::Rect ROI;		// Bounding box around object
	int classification;	// Object class
};

struct ClassificationResults
{
	boost::mutex mtx;	// Always lock first

	// Setters
	void clearObjects() { objects.clear(); }
	void addObject( boost::shared_ptr<FoundObject> obj) { objects.push_back(obj); }
	void setMask( boost::shared_ptr<cv::Mat> m ) { mask = m; }
	void setScene( pcl::PointCloud<pcl::PointXYZRGB>::Ptr s ) {scene = s;}

	// Getters
	boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > getObjects()
	{
		boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > copy( new std::vector< boost::shared_ptr<FoundObject> >(objects));
		return copy;
	}

	boost::shared_ptr<cv::Mat> getMask() { return mask; }

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getScene() {return scene;}

protected:
	std::vector< boost::shared_ptr<FoundObject> > objects;
	boost::shared_ptr<cv::Mat> mask;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene;
};