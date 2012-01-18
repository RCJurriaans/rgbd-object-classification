#include "stdafx.h"
#include "SegmentCloud.h"

#include <stdio.h>
#include <string>

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include <limits>
#include <ctime>
#include <math.h>

// Get Different Cloud types
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SegmentCloud::getNaNCloud()
{

	int nmax;
	cv::Mat BooleanMask( inputCloud->height, inputCloud->width, CV_8UC1);
	
	switch( getSegMethod() ){
	case SegBack:

		float point_backz;
		float point_imgz;

		nmax = inputCloud->width * inputCloud->height;
		std::cout << nmax << " points to check" << std::endl;

		for(int n=0; n < nmax ; n++ ){
			point_backz = backgroundCloud->at(n).z;
			point_imgz  = inputCloud->at(n).z;
				
			int i = n % inputCloud->width;
			int j = (n-i) % inputCloud->height;

			if(abs(point_backz-point_imgz)<threshold || point_imgz>distanceFilter){
				inputCloud->points[n].z = std::numeric_limits<float>::quiet_NaN();
				BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 0;
			}
			else {
				std::cout << "Width: " << i << ", Height: " << j << std::endl;
				BooleanMask.data[j*BooleanMask.step[0]+i*BooleanMask.step[1]] = 1;
			}

		}

		cvNamedWindow("TestMask", CV_WINDOW_AUTOSIZE); 
		cv::imshow("TestMask", BooleanMask);
		cv::waitKey();
		cvDestroyWindow("TestMask");

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

void SegmentCloud::setDistanceFilter(double distFilt)
{
	distanceFilter = distFilt;
}

int
	main (int argc, char** argv)
{
	SegmentCloud SC;

	std::string path_back;
	// std::cout << "Enter background pcd file path" << std::endl;
	// std::cin >> path_back;
	path_back = "Saves/Background/depth_0.pcd";

	std::string path_img;
	//std::cout << "Enter image pcd file path" << std::endl;
	//std::cin >> path_img;
	path_img = "Saves/SourMilk/depth_29.pcd";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_back, *cloud_back);
	SC.setBackgroundImage(cloud_back);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_img (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_img, *cloud_img);
	SC.setInputCloud(cloud_img);



	SC.setThreshold(0.05);
	SC.setDistanceFilter(2);

	SC.setSegMethod(SegmentCloud::SegBack);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud;
	pcl::RangeImage rangeImage;

	std::cout << "Starting " << cloud_img->width << " " << cloud_img->height <<  std::endl;

	time_t start = time(NULL);
	segmentCloud = SC.getNaNCloud();
	// SC.BooleanMask;
	time_t end = time(NULL);



	std::cout << "Time: " << difftime(end, start) << std::endl;
	std::cout << "Saving "  <<  std::endl;


	pcl::io::savePCDFileASCII ("Saves/test_pcd.pcd", *segmentCloud);
	std::cerr << "Saved " << cloud_img->points.size () << " data points to Saves/test_pcd.pcd." << std::endl;


	return (0);


}