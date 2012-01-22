
#include "stdafx.h"

#include "SegmentCloud.h"
#include "FeatureExtractor.h"
#include "RFClassifier.h"

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
		s_segCloud(segCloudPtr),
		extractor(new FeatureExtractor()),
		classifier(new RFClassifier())//"file", "dataname"))
	{
		cout << "ClassificationThread constructor" <<endl;
		extractor->loadCodebooks();
	}

	void operator()()
	{
		cout << "Classification Thread active" << endl;
		while(true)
		{
			// Get input from main thread
			inputMutex.lock();
			if (s_bgCloud != NULL) {
				cout << "Background received in classification thread" << endl;
				bgCloud.reset( new pcl::PointCloud<pcl::PointXYZRGB>(*s_bgCloud) );
				s_bgCloud.reset();
			}

			if (s_cloud != NULL) {
				//cout << "input cloud set"<< endl;
				cloud.reset( new pcl::PointCloud<pcl::PointXYZRGB>(*s_cloud));
				s_cloud.reset(); // s_cloud is read in other thread.
			}
			else {
				boost::thread::yield();
			}
			inputMutex.unlock();

			// Segment pointcloud
			if(cloud && bgCloud) {

				// Segment the object
				cout << "Segmenting.." << endl;
				//Sleep(5000);
				segmenter.setInputCloud(cloud);
				segmenter.setBackgroundImage(bgCloud);
				//segmenter.setThreshold(0.05);
				//segmenter.setDistanceFilter(3);
				//segmenter.setSegMethod(SegmentCloud::SegBack);
				segmenter.getNaNCloud();
				//segmenter.getROI();
				//segmenter.getWindowCloud();


				// Extract features
				//pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud);
				boost::shared_ptr<cv::Mat> img = FeatureExtractor::cloudToRGB(cloud);
				std::vector<bool> modes;
				modes.push_back(true);
				modes.push_back(false);
				modes.push_back(false);
				modes.push_back(false);
				modes.push_back(false);
				modes.push_back(false);
				cv::Mat features = extractor->extractFeatures( modes, *img );

				// Classify
				int predictedClass = classifier->predict(features);

				
				// Return output to main thread
				cout << "Sending classifier output to main thread"<<endl;
				outputMutex.lock();
				s_segCloud = cloud;
				outputMutex.unlock();
			}
			//Sleep(1);
			boost::thread::yield();
		}
	}
	
	pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

		return cloud_normals;
	}

	boost::shared_ptr<cv::Mat> calculateFPFH(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
	{
		// Create the FPFH estimation class, and pass the input dataset+normals to it
		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (cloud);
		fpfh.setInputNormals (normals);
		// alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		fpfh.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.05);

		// Compute the features
		fpfh.compute (*fpfhs);
		
		// Convert to cv::Mat (there's probably a faster way: memcpy innerloop; )
		boost::shared_ptr<cv::Mat> outMat( new cv::Mat(33, fpfhs->points.size(), CV_32FC1) );
		for (unsigned int p = 0; p < fpfhs->points.size(); p++)
		{
			for (unsigned int f = 0; f < 33; f++){
				outMat->data[outMat->step[0]*f + p] = fpfhs->at(p).histogram[f];
			}
		}
		return outMat;
	}
	
	boost::shared_ptr<FeatureExtractor> extractor;
	boost::shared_ptr<RFClassifier> classifier;

	// Threading
	boost::mutex& inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_bgCloud;

	boost::mutex& outputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr& s_segCloud;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	SegmentCloud segmenter;
};