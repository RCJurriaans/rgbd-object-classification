
#include "stdafx.h"
#include "ClassificationThread.h"

void ClassificationThread::run()
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
			//cout << "Segmenting \& Clasifying.." << endl;

			segmenter.setBackground(bgCloud);
			boost::shared_ptr<cv::Mat> mask = segmenter.getMask(cloud);
			cv::Rect ROI = segmenter.getROI(mask);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud = segmenter.getWindowCloud(ROI, cloud);
				
			// Extract features
			int predictedClass=0;
			/*boost::shared_ptr<cv::Mat> img = FeatureExtractor::cloudToRGB(cloud);
			std::vector<bool> modes;
			modes.push_back(true);
			modes.push_back(false);
			modes.push_back(false);
			modes.push_back(false);
			modes.push_back(false);
			modes.push_back(false);
			cv::Mat features = extractor->extractFeatures( modes, *img );

			// Classify
			predictedClass = classifier->predict(features);
			cout << "Predicted class: "<<predictedClass<<endl;*/

			boost::shared_ptr<FoundObject> object( new FoundObject(segmentedCloud, ROI, predictedClass) );

			
			// Return output to main thread
			results->mtx.lock();
				results->clearObjects();
				results->addObject(object);
				results->setMask(mask);
				results->setScene(cloud);
			results->mtx.unlock();

		}
		boost::thread::yield();
	}
}