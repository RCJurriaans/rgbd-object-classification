// DataDistributor.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Renderer.h"
#include <sstream>

using namespace xn;
using namespace cv;
using namespace std;

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

class DataDistributor
{
public:
	DataDistributor() :
		renderer(new Renderer(boost::bind(&DataDistributor::processInput, this, _1))),
		grabber(new pcl::OpenNIGrabber()),
		saveFolder(".\\saves\\"),
		numSaves(0),
		timeToSave(false)
	{
		// make callback function from member function
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> I_CB =
		  boost::bind(&DataDistributor::imageCB, this, _1);
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> D_CB =
		  boost::bind(&DataDistributor::depthCB, this, _1);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> CRGB_CB = 
			boost::bind(&DataDistributor::cloudRGBCB, this, _1);
//		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> ID_CB
//		  = boost::bind(&DataDistributor::imageDepthCB, this, _1, _2, _3);

		renderRGB = boost::bind(&Renderer::renderRGB, renderer, _1);
		renderDepth = boost::bind(&Renderer::renderDepth, renderer, _1);
		renderCloudRGB = boost::bind(&Renderer::renderCloudRGB, renderer, _1);

		// Connect callback functions. 
		boost::signals2::connection c = grabber->registerCallback(I_CB);
		boost::signals2::connection c2 = grabber->registerCallback(D_CB);
		boost::signals2::connection c3 = grabber->registerCallback(CRGB_CB);
		setRenderMode(renderRGBConnection, renderRGB, false);
	}

	~DataDistributor()
	{
		delete renderer;
		delete grabber;
	}

	void cloudRGBCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (timeToSave)
		{
			saveCloud(cloud);

			
			//int rows = cloud->points.size();
			//int cols = sizeof(pcl::PointXYZRGB);///sizeof(fpfhs.histogram[0]);
			cv::Mat rgbImage(480, 640, CV_8UC3);//, (void*)&cloud.points[0]);
			//uchar* rgbBuffer = new uchar[480*640*3];
			for (int x = 0; x < 640; x++)
			{
				for (int y = 0; y < 480; y++)
				{
					//rgbImage.data[rgbImage.step[0]*y + rgbImage.step[1]*x + rgbImage.step[3]*1] = cloud->
					//rgbImage.data[rgbImage.step[0]*y + rgbImage.step[1]*x + rgbImage.step[3]*2] = cloud.at(x, y);
					//rgbImage.data[rgbImage.step[0]*y + rgbImage.step[1]*x + rgbImage.step[3]*2] = cloud.at(x, y);
					 //M.data + M.step[0]*i + M.step[1]*j
					
				}
			}


			timeToSave = false;
			numSaves++;
		}
	}
	void imageDepthCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB, const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth, float constant)
	{
		
	}

	// These are required for handling events in each thread
	void imageCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB) { processInput( cvWaitKey(33) ); }
	void depthCB(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth) { processInput( cvWaitKey(33) ); }


	template <class T>
	void setRenderMode(boost::signals2::connection& con, boost::function<T>& callback, bool isCloud) {
		// Disconnect all rendering connections, and close windows
		renderRGBConnection.disconnect();
		renderDepthConnection.disconnect();
		//renderCloudRGBConnection.disconnect();
		cvDestroyWindow("RGB Display");
		cvDestroyWindow("Depth Display");
		
		if (isCloud) 
			renderer->openCloudDisplay();
		else
			renderer->closeCloudDisplay();
		//cvWaitKey(33);

		// Register the new callback:
		con = grabber->registerCallback(callback);
	}

	bool saveCloud( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud )
	{
		stringstream depth_filename;
		depth_filename << saveFolder << "depth_" << numSaves << ".pcd";
		return pcl::io::savePCDFile(depth_filename.str(), *cloud.get(), true);
		//cout << "Saved depth and RGB data. Index: " << numSaves << std::endl;
		
		//TODO template
	}

	bool saveRGB( const Mat& rgbImage)
	{
		stringstream  rgb_filename;
		rgb_filename << saveFolder << "rgb_" << numSaves << ".bmp";
		if(!cv::imwrite(rgb_filename.str(), rgbImage)) {
			cout << "Could not save: " << rgb_filename.str().c_str();
			return false;
		}
		return true;
	}

	void processInput( int key )
	{
		switch(key)
		{
		case -1:
			return;
		case 32: // space
			timeToSave = true;
			break;
		case 27: // escape
			exit(0);
			break;
		case '1':
			setRenderMode(renderRGBConnection, renderRGB, false);
			break;
		case '2':
			setRenderMode(renderDepthConnection, renderDepth, false);
			break;
		case '3':
			//renderer->openCloudDisplay();
			setRenderMode(renderCloudRGBConnection, renderCloudRGB, true);
			break;
		default:
			return;
		}
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
	
	pcl::Grabber* grabber;
	Renderer* renderer;
	string saveFolder;
	int numSaves;
	bool timeToSave;

	// Callbacks
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> renderRGB;
	boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> renderDepth;
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)> renderCloudRGB;

	// Callback connections
	boost::signals2::connection renderRGBConnection;
	boost::signals2::connection renderDepthConnection;
	boost::signals2::connection renderCloudRGBConnection;
};

int main ()
{
	DataDistributor d;
	d.run();
	return 0;
}
