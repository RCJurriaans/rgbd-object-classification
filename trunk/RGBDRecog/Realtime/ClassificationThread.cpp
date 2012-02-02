
#include "stdafx.h"
#include "ClassificationThread.h"

void ClassificationThread::run()
{
	cout << "Classification Thread active" << endl;
	bool classify = false;

	string newClassName;
	bool trainNow = false;
	bool addDataPointNow = false;
	bool makeNewClass = false;
	int currentLearnClass = 0;

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

		if (s_trainNow) {
			trainNow = true;
			s_trainNow = false;
		} else { trainNow = false; }
		if (s_addDataPointNow) {
			addDataPointNow = s_addDataPointNow;
			s_addDataPointNow = false;
		} else {addDataPointNow = false; }
		if (s_makeNewClass) {
			makeNewClass = s_makeNewClass;
			s_makeNewClass = false;
		} else {makeNewClass = false;}
		inputMutex.unlock();


		// Segment pointcloud
		if(cloud){// && bgCloud) {

			//segmenter.setBackground(bgCloud);
			boost::shared_ptr<pcl::PointIndices> objectInliers(new pcl::PointIndices);
			boost::shared_ptr<pcl::PointIndices> planeInliers(new pcl::PointIndices);
			boost::shared_ptr<pcl::ModelCoefficients> planeCoeffs(new pcl::ModelCoefficients);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
			boost::shared_ptr<cv::Mat> mask = segmenter.getMask(cloud, objectInliers, planeInliers, planeCoeffs, cloud_filtered);
			boost::shared_ptr<std::vector<cv::Rect> > ROIs = segmenter.getROIS(mask);
			boost::shared_ptr<cv::Mat> img = FeatureExtractor::cloudToRGB(cloud);

			// Process each found object
			vector<boost::shared_ptr<FoundObject> > objs;
			int i = 0;
			for(vector<cv::Rect>::iterator ROI = ROIs->begin(); ROI != ROIs->end(); ROI++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud = segmenter.getWindowCloud(*ROI, cloud);
				//boost::shared_ptr<cv::Mat> croppedMask( new cv::Mat((*mask)(*ROI)) );
				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr unorgSegCloud =
				//	segmenter.getUnorgCloud(croppedMask, segmentedCloud);
				
				
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr unorgSegCloud =
					segmenter.getUnorgCloud(*mask, cloud, *ROI);
				//pcl::PointCloud<pcl::PointXYZRGB>::Ptr unorgSegCloud( new pcl::PointCloud<pcl::PointXYZRGB>() );
				//std::vector<int> a;
				//pcl::removeNaNFromPointCloud(*segmentedCloud,*unorgSegCloud,a);
				
				//cout << "Window cloud size: "<< segmentedCloud->width << " " << segmentedCloud->height << endl;
				//cout << "ROI size: " << ROI->width << " " << ROI->height << endl;
				//cout << "Cropped mask size" << croppedMask->cols << " " << croppedMask->rows << endl;
				//cout << "Unorg seg cloud size: " << unorgSegCloud->size() << endl;
				pcl::ModelCoefficients coeffs( segmenter.getSmallestBoundingBox( unorgSegCloud ));//cloud, *mask, *ROI));
				//pcl::ModelCoefficients coeffs( segmenter.getSmallestBoundingBox( cloud, *mask, *ROI));


				////////
				boost::shared_ptr<cv::Mat> maskOut;
				cv::Rect ROIOut;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new  (new pcl::PointCloud<pcl::PointXYZRGB>);
				cloud_new = segmenter.tijmenLikesHacking(maskOut, ROIOut, cloud);

				std::cout << cloud_new->size() << std::endl;
				//pcl::ModelCoefficients coeffs;
				if(cloud_new->size()!=0){
					coeffs = segmenter.getSmallestBoundingBox(cloud_new);
				}
				///////
				
				//pcl::ModelCoefficients coeffs( segmenter.getCoefficients(*ROI, cloud, mask) ); // Axis aligned bb
				
				int predictedClassRF = -1;

				//if (classify) {
					// Extract features
					if( !(ROI->x == 0 && ROI->y == 0 && ROI->width == 0 && ROI->height == 0)) {
						//cout << "valid ROI"<<endl;

						// Classify using RF classifier
						//cv::Mat features = extractor->extractFeatures( modes, (*img)(*ROI), unorgSegCloud, coeffs, (*mask)(*ROI) );
						cv::Mat features = extractor->extractFeatures( modes, (*img)(ROIOut), cloud_new, coeffs, *maskOut );
						if (features.rows != 0 && features.cols != 0)
							predictedClassRF = rf->predict(features);

						// Online Learning
						if( i ==0 ) {
							if( addDataPointNow ) {
								cout << "Classification thread: adding data point to training set" << endl;
								if( makeNewClass ) {
									currentLearnClass = rf->addTrainingPoint(features, -1);				
								} else {
									rf->addTrainingPoint(features, currentLearnClass);
								}
							}
							if( trainNow ) {
								rf->trainTree();
							}
						}
					}
				
					
				//}
				boost::shared_ptr<FoundObject> object( new FoundObject(segmentedCloud, *ROI, coeffs, predictedClassRF) );
				objs.push_back(object);	
				i++;
				break;
			}

			


			// Return output to main thread
			results->mtx.lock();
				results->clearObjects();
				for(int i=0; i < objs.size();i++)
					results->addObject(objs.at(i));
				results->setMask(mask);
				results->setObjectInliers(objectInliers);
				results->setPlaneInliers(planeInliers);
				results->setPlaneCoeffs(planeCoeffs);
				results->setSceneFiltered(cloud_filtered);
				results->setScene(cloud);
				results->hasNew=true;

				// Get
				classify = results->classify;
			results->mtx.unlock();

		}
		//Sleep(2000);
		boost::thread::yield();
	}
}