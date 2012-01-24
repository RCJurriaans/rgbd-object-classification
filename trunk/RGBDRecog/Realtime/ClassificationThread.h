
#include "stdafx.h"

#include "SegmentCloud.h"
#include "FeatureExtractor.h"
#include "RFClassifier.h"
#include "ClassificationResults.h"

class ClassificationThread
{
public:	
	ClassificationThread( boost::mutex& inmutex,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloudPtr,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& bgCloudPtr,
						  boost::shared_ptr<ClassificationResults> res) :
		inputMutex(inmutex),
		s_cloud(cloudPtr),
		s_bgCloud(bgCloudPtr),
		results(res)
		,extractor(new FeatureExtractor())
		,classifier(new RFClassifier())//"D:\\randomforestdata100000.yml", "randomforestdata100000"))//"file", "dataname"))
	{
		//extractor->loadCodebooks();
		//segmenter.setSegMethod(SegmentCloud::SegPlane);
	}

	void run();
	void setBackground( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& bgCloud);

	// System components
	boost::shared_ptr<FeatureExtractor> extractor;
	boost::shared_ptr<RFClassifier> classifier;

	// Threading
	boost::mutex& inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_bgCloud;

	boost::shared_ptr<ClassificationResults> results;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	SegmentCloud segmenter;

private:
	ClassificationThread( const ClassificationThread& other ); // non construction-copyable
    ClassificationThread& operator=( const ClassificationThread& ); // non copyable
};