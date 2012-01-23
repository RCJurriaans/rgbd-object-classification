

#include "stdafx.h"
#include "Renderer.h"

void Renderer::setResults( cv::Rect ROI )
{
	cout << "Renderer input ROI: " << ROI.x << " " <<ROI.y << " " << ROI.width << " " << ROI.height << endl;
	mtx.lock();
		vizROI = ROI;
	mtx.unlock();
}


void Renderer::renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniBGR) 
{
	oniBGR->fillRGB(640, 480, RGBImage.data);
	cv::cvtColor( RGBImage, RGBImage, cv::COLOR_BGR2RGB ); //CV_BGR2RGB );

	/*cv::Rect ROI;
	cout << "rendering rgb.."<<endl;
	//rectangle(Mat& img, Rect r, const Scalar& color, int thickness=1, int lineType=8, int shift=0)¶
	mtx.lock();
		ROI = vizROI;
	mtx.unlock();*/
	results->mtx.lock();
	boost::shared_ptr< std::vector< boost::shared_ptr<FoundObject> > > objects = results->getObjects();
	cv::Rect ROI;
	boost::shared_ptr<cv::Mat> mask = results->getMask();
	if (objects->size() > 0) {
		cout <<"results received in renderer!"<<endl;
		ROI = objects->at(0)->ROI;
	}
	results->mtx.unlock();

	cout << "Renderer render ROI: " << ROI.x << " " << ROI.y << " " << ROI.width << " " << ROI.height << endl;
	cv::rectangle(RGBImage, ROI, cv::Scalar(255,0,0));
	cv::imshow( "RGB Display", RGBImage);

	if( mask ) {
		cout << "Rendering mask" << endl;
		cv::imshow( "Mask display", *mask );
	}
}

void Renderer::renderOpenCVRGB(const boost::shared_ptr<cv::Mat>& RGBImage )
{
	cv::imshow( "RGB Display", *RGBImage);
}

void Renderer::renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth)
{
	oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
	//cv::rectangle(depthImage, ROI, cv::Scalar(100));
	cv::imshow("Depth Display", depthImage);
}

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	if(e.keyDown()) {
		(*processInput)(e.getKeyCode());
	}
}

void Renderer::renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, //without const, could do without copy
							  cv::Rect ROI)
{
	/*
	if (!cloudViewerOpen) {
		if (viewer != NULL) {
			delete viewer;
			viewer = NULL;
		}
	}
	else {
		cout << "rendering cloud" << endl;
		if (viewer == NULL)
		{
			viewer = new pcl::visualization::CloudViewer("Cloud Display");
			viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		}
		viewer->showCloud (cloud);
	}*/
	//viewer->showCloud (cloud);
	//processInput( cvWaitKey(1) );

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloud, *cloudCopy);
	
	mtx.lock();
		vizCloud = cloudCopy;
		vizROI = ROI;
	mtx.unlock();
}

void Renderer::renderCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy( new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZRGB>(*cloud, *cloudCopy);
	
	mtx.lock();
		vizCloud = cloudCopy;
	mtx.unlock();
}


void Renderer::closeCloudDisplay()
{
	cloudViewerOpen = false; 
}

void Renderer::openCloudDisplay()
{
	cloudViewerOpen = true;
}

// This method runs on the visualization thread
void Renderer::visCallback(pcl::visualization::PCLVisualizer& vis)
{
	/*

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	cv::Rect ROI;

	mtx.lock();
		if( !vizCloud ){
			mtx.unlock();
			boost::this_thread::sleep (boost::posix_time::milliseconds (1));
			return;
		}
		tempCloud.swap(vizCloud);
		ROI = vizROI;
	mtx.unlock();
	
	if (!vis.updatePointCloud (tempCloud))
	{	
		vis.addPointCloud (tempCloud);
		vis.resetCameraViewpoint ();
	}
	
	if(!testCloud){
		cout << "test cloud null"<<endl;
		return;
	}
	cout << "rendering testcloud" << endl;
	*/
}