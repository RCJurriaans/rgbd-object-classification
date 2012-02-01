
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
		if(cloud){// && bgCloud) {

			//segmenter.setBackground(bgCloud);
			boost::shared_ptr<pcl::PointIndices> inliers(new pcl::PointIndices);
			boost::shared_ptr<cv::Mat> mask = segmenter.getMask(cloud, inliers);
			boost::shared_ptr<std::vector<cv::Rect> > ROIs = segmenter.getROIS(mask);
			boost::shared_ptr<cv::Mat> img = FeatureExtractor::cloudToRGB(cloud);

			// Process each found object
			vector<boost::shared_ptr<FoundObject> > objs;
			for(vector<cv::Rect>::iterator ROI = ROIs->begin(); ROI != ROIs->end(); ROI++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud = segmenter.getWindowCloud(*ROI, cloud);
				//boost::shared_ptr<cv::Mat> croppedMask( new cv::Mat((*mask)(*ROI)) );
				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr unorgSegCloud =
				//	segmenter.getUnorgCloud(croppedMask, segmentedCloud);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr unorgSegCloud =
					segmenter.getUnorgCloud(*mask, cloud, *ROI);

				cout << "Window cloud size: "<< segmentedCloud->width << " " << segmentedCloud->height << endl;
				cout << "ROI size: " << ROI->width << " " << ROI->height << endl;
				//cout << "Cropped mask size" << croppedMask->cols << " " << croppedMask->rows << endl;
				cout << "Unorg seg cloud size: " << unorgSegCloud->size() << endl;
				pcl::ModelCoefficients coeffs( segmenter.getSmallestBoundingBox( unorgSegCloud ));//cloud, *mask, *ROI));
				//pcl::ModelCoefficients coeffs( segmenter.getSmallestBoundingBox( cloud, *mask, *ROI));

				/*boost::shared_ptr<cv::Mat> maskOut;
				cv::Rect ROIOut;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new  (new pcl::PointCloud<pcl::PointXYZRGB>);
				cloud_new = segmenter.tijmenLikesHacking(maskOut, ROIOut, segmentedCloud);

				std::cout << cloud_new->size() << std::endl;
				pcl::ModelCoefficients coeffs;
				if(cloud_new->size()!=0){
					coeffs = segmenter.getSmallestBoundingBox(cloud_new);
				}*/

				
				//pcl::ModelCoefficients coeffs( segmenter.getCoefficients(*ROI, cloud, mask) ); // Axis aligned bb
				
				int predictedClassRF = -1;
				int predictedClassNN = -1;

				// Extract features
				if( !(ROI->x == 0 && ROI->y == 0 && ROI->width == 0 && ROI->height == 0)) {
					//cout << "valid ROI"<<endl;

					// Classify using RF classifier
					cv::Mat features = extractor->extractFeatures( modes, (*img)(*ROI), cloud, *mask );
					if (features.rows != 0 && features.cols != 0)
						predictedClassRF = rf->predict(features);

					// Classify using NBNN:
					//vector<cv::Mat> rawFeatures = extractor->extractRawFeatures(modes, (*img)(*ROI), cloud, *mask);
					//if( rawFeatures.size() > 0 ) {
					//	predictedClassNN = nn->classify(features);
					//}
					//cout << "num raw features " << rawFeatures.size() <<endl;

				}
				
			

				//cout << "Predicted class (RF): "<< predictedClassRF << endl;
				//cout << "Predicted class (NN): "<< predictedClassNN << endl;
				boost::shared_ptr<FoundObject> object( new FoundObject(segmentedCloud, *ROI, coeffs, predictedClassRF) );
				objs.push_back(object);
				
			//	break;
			}

			// Return output to main thread
			results->mtx.lock();
				results->clearObjects();
				for(int i=0; i < objs.size();i++)
					results->addObject(objs.at(i));
				results->setMask(mask);
				results->setInliers(inliers);
				results->setScene(cloud);
				results->hasNew=true;
			results->mtx.unlock();

		}
		//Sleep(2000);
		boost::thread::yield();
	}
}