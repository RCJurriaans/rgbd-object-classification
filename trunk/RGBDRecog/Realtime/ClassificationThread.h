
#include "stdafx.h"

#include "SegmentCloud.h"
#include "FeatureExtractor.h"
#include "RFClassifier.h"
#include "ClassificationResults.h"
#include "NBNN.h"

class ClassificationThread
{
public:	
	ClassificationThread( boost::mutex& inmutex,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloudPtr,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& bgCloudPtr,
						  boost::shared_ptr<ClassificationResults> res,
						  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& newCloudPtr,
						  std::string& newClassNamePtr) :
		inputMutex(inmutex),
		s_cloud(cloudPtr),
		s_bgCloud(bgCloudPtr),
		results(res)
		,extractor(new FeatureExtractor())
		,rf(new RFClassifier())
		,nn(new NBNN())
		,s_newCloud(newCloudPtr)
		,s_newClassName(newClassNamePtr)
	{
		std::string datadir = getenv("RGBDDATA_DIR");

		extractor->loadCodebooks();
		rf->read(datadir + "\\randomforestdata_100000011.yml", "randomforestdata_100000011");
		//segmenter.setSegMethod(SegmentCloud::SegPlane);

		//cv::FileStorage f("NBNN_10000001.yml", cv::FileStorage::READ);
		//nn->read(f);


		modes.push_back(false);
		modes.push_back(false);
		modes.push_back(false);
		modes.push_back(false);
		modes.push_back(false);
		modes.push_back(false);
		modes.push_back(true);
		modes.push_back(true);

		segmenter.setSegMethod(SegmentCloud::SegPlane);
	}

	void run();
	void setBackground( pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& bgCloud);

	// System components
	boost::shared_ptr<FeatureExtractor> extractor;
	boost::shared_ptr<RFClassifier> rf;
	boost::shared_ptr<NBNN> nn;
	SegmentCloud segmenter;

	// State variables
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr bgCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	std::vector<bool> modes;

	// Threading
	boost::mutex& inputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_bgCloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& s_newCloud;
	  string& s_newClassName;

	boost::shared_ptr<ClassificationResults> results;

private:
	ClassificationThread( const ClassificationThread& other ); // non construction-copyable
    ClassificationThread& operator=( const ClassificationThread& ); // non copyable
};