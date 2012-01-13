// DataDistributor.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Renderer.h"

using namespace xn;
using namespace cv;

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif


class DataDistributor
{
public:
	DataDistributor() :
		RGBImage(Mat(480, 640, CV_8UC3)),//Mat(cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3))), // Weird init is necessary for data layout
		depthImage(Mat(480,640, CV_32FC1)),// Mat(cvCreateImage(cvSize(640,480), IPL_DEPTH_16S, 1))),
		renderer(),
		grabber(new pcl::OpenNIGrabber())
	{
		// make callback function from member function
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> I_CB =
		  boost::bind(&DataDistributor::imageCB, this, _1);
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> ID_CB
		  = boost::bind(&DataDistributor::imageDepthCB, this, _1, _2, _3);

		// connect callback function for desired signal. In this case its a point cloud with color values
		boost::signals2::connection I_c = grabber->registerCallback(I_CB);
		boost::signals2::connection ID_c = grabber->registerCallback(ID_CB);
	}

	~DataDistributor()
	{
		delete renderer;
		delete grabber;
	}

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
	  //if (!viewer.wasStopped())
       //  viewer.showCloud (cloud);
    //static unsigned count = 0;
    //static double last = pcl::getTime ();
    //if (++count == 30)
    //{
     // double now = pcl::getTime ();
      //std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      //count = 0;
      //last = now;
   // }
	}
	void imageDepthCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB, const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth, float constant)
	{
		/*oniRGB->fillRGB(640, 480, RGBImage.data);
		cvtColor( RGBImage, RGBImage, CV_BGR2RGB );
		cvNamedWindow("RGB Display", CV_WINDOW_AUTOSIZE);
		imshow( "RGB Display", RGBImage);
		cvWaitKey(33);*/
		
		//oniDepth->fillDepthImageRaw(640, 480, (unsigned short*)depthImage.data);
		oniDepth->fillDepthImage(640, 480, (float*)depthImage.data);
		cvNamedWindow("Depth Display", CV_WINDOW_AUTOSIZE);
		imshow("Depth Display", depthImage);
		cvWaitKey(1);
	}

	void imageCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB)
	{
		//oniRGB->fillRaw(RGBImage.data);
		oniRGB->fillRGB(640, 480, RGBImage.data);
		cvtColor( RGBImage, RGBImage, CV_BGR2RGB );
		//cvNamedWindow("RGB Display", CV_WINDOW_AUTOSIZE);
		imshow( "RGB Display", RGBImage);
		cvWaitKey(1);
	}


	void run ()
	{
		// start receiving point clouds
		grabber->start();

		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while (true)
			sleep(1);

		// stop the grabber
		grabber->stop();
	}

protected:
	Mat RGBImage;
	Mat depthImage;
	pcl::Grabber* grabber;
	Renderer* renderer;
};

int main ()
{
	DataDistributor d;
	d.run();
	return 0;
}
