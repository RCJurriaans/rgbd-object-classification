#include "stdafx.h"
#include "SegmentCloud.h"

#include <stdio.h>
#include <string>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <limits>
#include <ctime>
#include <math.h>

#include <pcl/range_image/range_image.h>


// Get Different Cloud types
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentCloud::getNaNCloud()
{

	int nmax;

	switch( getSegMethod() ){
	case SegBack:
		float point_backz;
		float point_imgz;

		nmax = inputCloud->width * inputCloud->height;

		for(int n=0; n < nmax ; n++ ){
			point_backz = backgroundCloud->at(n).z;
			point_imgz  = inputCloud->at(n).z;

			if(abs(point_backz-point_imgz)<0.05){
				inputCloud->points[n].z = std::numeric_limits<float>::quiet_NaN() ;			
			}
		}
	case SegObj:
		std::cout << "No such method yet" << std::endl;
	case SegPlane:
		std::cout << "No such method yet" << std::endl;
	case SegNormHist:
		std::cout << "No such method yet" << std::endl;
	}

	return inputCloud;

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

void SegmentCloud::getMask()
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

int
	main (int argc, char** argv)
{
	SegmentCloud SC;

	std::string path_back;
	std::cout << "Enter background pcd file path" << std::endl;
	std::cin >> path_back;

	std::string path_img;
	std::cout << "Enter image pcd file path" << std::endl;
	std::cin >> path_img;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_back, *cloud_back);
	SC.setBackgroundImage(cloud_back);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_img (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_img, *cloud_img);
	SC.setInputCloud(cloud_img);

	std::cout << "Starting " << cloud_img->width << " " << cloud_img->height <<  std::endl;

	time_t start = time(NULL);
	cloud_img = SC.getNaNCloud();
	time_t end = time(NULL);

	std::cout << "Time: " << difftime(end, start) << std::endl;
	std::cout << "Saving "  <<  std::endl;


	// Something wrong with pointer here
	pcl::io::savePCDFileASCII ("Saves/test_pcd.pcd", *cloud_img);
	std::cerr << "Saved " << cloud_img->points.size () << " data points to Saves/test_pcd.pcd." << std::endl;


	return (0);


}