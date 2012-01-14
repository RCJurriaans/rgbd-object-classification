

#include "stdafx.h"
#include "Renderer.h"

using namespace xn;
using namespace cv;



void Renderer::renderRGB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB) 
{
	oniRGB->fillRGB(640, 480, RGBImage.data);
	cvtColor( RGBImage, RGBImage, CV_BGR2RGB );
	imshow( "RGB Display", RGBImage);
}

void Renderer::renderDepth(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth)
{
	oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
	imshow("Depth Display", depthImage);
}

void keyboardCB(const pcl::visualization::KeyboardEvent& e, void* cookie)
{
	cout << ".." << e.getKeySym() << endl;
	boost::function<void (int)>* processInput = (boost::function<void (int)>*)cookie;
	(*processInput)(e.getKeyCode());
	
	
	//if (e.keyDown()) {
	//	cout << "down" << e.getKeySym() << endl;
	//	
	//}
}

void Renderer::renderCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
{
	cout << "cloud cb ";
	if (!cloudViewerOpen) {
		
		cout << "viewer closed. ";
		if (viewer != NULL) {
			cout << " Deleting viewer";
			delete viewer;
			viewer = NULL;
		}
		
	}
	else {
		cout << "viewer OPEN";
		if (viewer == NULL)
		{
			cout << "opening viewer..";
			viewer = new pcl::visualization::CloudViewer("Cloud Display");
			viewer->registerKeyboardCallback( &keyboardCB, &processInput );
		}
		viewer->showCloud (cloud);
	}
	cout << endl;
}




void Renderer::closeCloudDisplay()
{
	cout << "closing" << endl;
	cloudViewerOpen = false; 
	//if(viewer != NULL)
	//{

	//	cout << "closing!" << endl;
	//	destroyCloudViewer = true;
		//delete viewer;
		//cout << (viewer == NULL);
		//viewer = NULL;
	//}
}

void Renderer::openCloudDisplay()
{
	cout << "opening" << endl;
	cloudViewerOpen = true;
}
