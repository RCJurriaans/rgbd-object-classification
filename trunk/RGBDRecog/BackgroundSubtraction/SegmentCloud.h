
#include "stdafx.h"

class SegmentCloud
{


	// Cloud to segment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr backgroundCloud;
	// Optional Background image (only necessary for background subtraction)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;


public:
		// Method of Segmentation
	enum SegmentationMethod { SegBack, SegObj, SegPlane, SegNormHist };
	SegmentationMethod crtMethod;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr NaNCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr WindowCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr UnorgCloud;

	double threshold;
	double distanceFilter;

	// Get Different Cloud types
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getNaNCloud();
	void getWindowCloud();
	void getUnorgCloud();

	// Choose segmentation method
	SegmentationMethod getSegMethod();
	void setSegMethod(SegmentationMethod);

	// Get different masks
	void getROI();
	void getMask();

	// Set necessary clouds
	void setBackgroundImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);

	void SegmentCloud::setThreshold(double);
	void SegmentCloud::setDistanceFilter(double);
protected:

private:

};