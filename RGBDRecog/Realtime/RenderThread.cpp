
#include "stdafx.h"
#include "RenderThread.h"

/*
void RenderThread::run()
{
	cout << "Render thread running" << endl;
	
	//boost::shared_ptr<pcl::visualization::PCLVisualizer>
	//viewer.reset( new pcl::visualization::PCLVisualizer() );
	//pcl::visualization::PCLVisualizer* viewer = new pcl::visualization::PCLVisualizer();
	viewer->setBackgroundColor (0, 0, 0);

		//pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr c( new pcl::PointCloud<pcl::PointXYZRGB>() );
		//viewer->addPointCloud<pcl::PointXYZRGB> (c);
		viewer->addPointCloud(cloud);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
	//viewer->spin();
	//cout << "stopped spinning" << endl;

	while (!viewer->wasStopped ())
	{
		cout << "Spinning.." << endl;
		inputMutex.lock();

		//if(!viewer->updatePointCloud(cloud))
		//	viewer->addPointCloud(cloud);


		viewer->spinOnce (100);

		inputMutex.unlock();

		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		//Sleep(1);
		
	}
}

void RenderThread::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToShow)
{
	cout << "Setting input for rendering" << endl;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr* cloudCopy = new pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>());
		//( cloudToShow );
	pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZRGB>(*cloudToShow, **cloudCopy);
		//(*cloudToShow, *cloudCopy);

	//cout << "Cloud copied" <<endl;
	inputMutex.lock();
	//cout<<"mutex locked"<<endl;
	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr c(new pcl::PointCloud<pcl::PointXYZRGB>());

	viewer->removeAllPointClouds();
	cout << "removed all" <<endl;
	viewer->addPointCloud(c);

	//this->cloud.reset(cloudCopy.get());
	//cout<<"cloud reset"<<endl;
	//viewer->getGeometryHandlerIndex("aaa");
	//cout << viewer.use_count() << endl;
	//if(!viewer->updatePointCloud(*cloudCopy))
	//	viewer->addPointCloud(*cloudCopy);

	inputMutex.unlock();

}*/