
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

			segmenter.setBackground(bgCloud);
			boost::shared_ptr<cv::Mat> mask = segmenter.getMask(cloud);
			mask = segmenter.denoizeMask( mask );
			boost::shared_ptr<std::vector<cv::Rect> > ROIs = segmenter.getROIS(mask);
			boost::shared_ptr<cv::Mat> img = FeatureExtractor::cloudToRGB(cloud);

			

			//for (int obj = 0; obj < ROIs->size(); obj++)

			vector<boost::shared_ptr<FoundObject> > objs;

			for(vector<cv::Rect>::iterator ROI = ROIs->begin(); ROI != ROIs->end(); ROI++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud = segmenter.getWindowCloud(*ROI, cloud);
			
				pcl::ModelCoefficients coeffs( segmenter.getCoefficients(*ROI, cloud, mask) );

				int predictedClassRF = -1;
				int predictedClassNN = -1;

				// Extract features
				if( !(ROI->x == 0 && ROI->y == 0 && ROI->width == 0 && ROI->height == 0)) {
					//cout << "valid ROI"<<endl;

					// Classify using RF classifier
					cv::Mat features = extractor->extractFeatures( modes, *img, *ROI );
					if (features.rows != 0 && features.cols != 0)
						predictedClassRF = rf->predict(features);

					// Classify using NBNN:
					vector<cv::Mat> rawFeatures = extractor->extractRawFeatures(modes, *img, *ROI);
					if( rawFeatures.size() > 0 ) {
						predictedClassNN = nn->classify(features);
					}
					//cout << "num raw features " << rawFeatures.size() <<endl;

				}

			

				//cout << "Predicted class (RF): "<< predictedClass << endl;
				//cout << "Predicted class (NN): "<< predictedClass2 << endl;
				boost::shared_ptr<FoundObject> object( new FoundObject(segmentedCloud, *ROI, coeffs, predictedClassNN) );
				objs.push_back(object);
				
				//break;
			}

			// Return output to main thread
			results->mtx.lock();
				results->clearObjects();
				for(int i=0; i < objs.size();i++)
					results->addObject(objs.at(i));
				results->setMask(mask);
				results->setScene(cloud);
			results->mtx.unlock();

		}
		//Sleep(2000);
		boost::thread::yield();
	}
}