
#include "stdafx.h"

#include "SegmentCloud.h"

class ClassificationThread
{
public:	
	ClassificationThread( boost::mutex& mutex,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudPtr,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& bgCloudPtr) :
		RGBXYZCloudMutex(mutex),
		s_cloud(cloudPtr),
		s_bgCloud(bgCloudPtr)
	{
		v = new pcl::visualization::CloudViewer("Cloud Viewer2");
	}

	void operator()()
	{

		//cout << "Classification Thread active" << endl;
		while(true)
		{
			// Get input from main thread
			RGBXYZCloudMutex.lock();
			cloud = s_cloud;
			bgCloud = s_bgCloud;
			RGBXYZCloudMutex.unlock();

			// Segment pointcloud
			cout << "Segmenting.." << endl;
			segmenter.setInputCloud(cloud);
			segmenter.setBackgroundImage(bgCloud);
			segmenter.setDistanceFilter(3);
			segmenter.setThreshold(0.05);
			segmenter.getNaNCloud();

			v->showCloud(cloud);

			// Classify data
			//TODO

			// Return output to main thread
			

			Sleep(1);
		}
	}
	pcl::visualization::CloudViewer* v;

	boost::mutex& RGBXYZCloudMutex;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& s_cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr& s_bgCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgCloud;

	SegmentCloud segmenter;
};