
#include "stdafx.h"

#include "SegmentCloud.h"

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

class ClassificationThread
{
public:	
	ClassificationThread( boost::mutex& inmutex,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloudPtr,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& bgCloudPtr,
						  boost::mutex& outmutex,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segCloudPtr) :
		inputMutex(inmutex),
		s_cloud(cloudPtr),
		s_bgCloud(bgCloudPtr),
		outputMutex(outmutex),
		s_segCloud(segCloudPtr)
	{
		//v = new pcl::visualization::CloudViewer("Cloud Viewer2");
	}

	void operator()()
	{
		//cout << "Classification Thread active" << endl;
		while(true)
		{
			// Get input from main thread
			inputMutex.lock();
			if (s_bgCloud != NULL) {
				cout << "UPDATED!!" << endl;
				bgCloud.reset( new pcl::PointCloud<pcl::PointXYZRGB>(*s_bgCloud) );
				s_bgCloud.reset();
			}

			if (s_cloud != NULL) {
				cout << "input cloud set"<< endl;
				cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB>(*s_cloud));
				//s_cloud.reset(); // s_cloud is read in other thread.
			}
			inputMutex.unlock();

			// Segment pointcloud
			if(cloud && bgCloud) {

				cout << "Segmenting.." << endl;
				segmenter.setInputCloud(cloud);
				segmenter.setBackgroundImage(bgCloud);
				segmenter.setThreshold(0.05);
				//segmenter.setDistanceFilter(3);
				segmenter.setSegMethod(SegmentCloud::SegBack);
				segmenter.getUnorgCloud();

				// Classify data
				//TODO
				
				//pcl::io::savePCDFile("testseg.pcd", *cloud.get(), true);
				//cout << "saved" << endl;

				// Return output to main thread
				outputMutex.lock();
				s_segCloud = cloud;
				outputMutex.unlock();
			}
			Sleep(1);
			//boost::thread::yield();
		}
	}
	/*
	void calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (cloud);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (0.03);

		// Compute the features
		ne.compute (*cloud_normals);
	}

	void calculateFPFH(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
	{

		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

		// Create the FPFH estimation class, and pass the input dataset+normals to it
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (cloud);
		fpfh.setInputNormals (normals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
		fpfh.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.05);

		// Compute the features
		fpfh.compute (*fpfhs);

	}
	*/
	boost::mutex& inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_bgCloud;

	boost::mutex& outputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& s_segCloud;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	SegmentCloud segmenter;
};