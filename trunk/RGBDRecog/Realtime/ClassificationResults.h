
#pragma once

#include "stdafx.h"

// Immutable (thus thread-safe) object for storing segmented and classified objects
// In theory, objects pointed to can be unsafe, but they should not be accessed 
struct FoundObject
{
	FoundObject(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c,
					cv::Rect bb2d,
					pcl::ModelCoefficients bb3d,
					int cl ) :
		cloud(c), boundingBox2D(bb2d), boundingBox3D(bb3d), classification(cl)
	{}
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud() {return cloud;}
	cv::Rect getBoundingBox2D() {return boundingBox2D;}
	pcl::ModelCoefficients getBoundingBox3D() {return boundingBox3D;}
	int getClassification() {return classification;}

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;				// Cloud of the segmented object
	cv::Rect boundingBox2D;										// 2D Bounding box around object
	pcl::ModelCoefficients boundingBox3D;	// 3D Bounding box 
	int classification;											// Object class

};

struct ClassificationResults
{
	boost::mutex mtx;	// Always lock first

	// Setters
	void clearObjects() { objects.clear(); }
	void addObject( boost::shared_ptr<FoundObject> obj) { objects.push_back(obj); }
	//void addObjects( 
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