// DataDistributor.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Renderer.h"
#include "NBNN.h"
#include "ClassificationThread.h"
#include "SegmentCloud.h"

//using namespace xn;
//using namespace std;

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
		timeToSave(false),
		NBNNClassifier(new NBNN())
	{
		// make callback function from member function
		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> I_CB =
		  boost::bind(&DataDistributor::imageCB, this, _1);
		boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> D_CB =
		  boost::bind(&DataDistributor::depthCB, this, _1);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)> C_CB = 
			boost::bind(&DataDistributor::cloudCB, this, _1);
		saveCB = boost::bind(&DataDistributor::saveFrameCB, this, _1); // Don't want to register this one yet, slows things down.
//		boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)> ID_CB
//		  = boost::bind(&DataDistributor::imageDepthCB, this, _1, _2, _3);

		renderRGB = boost::bind(&Renderer::renderRGB, renderer, _1);
		renderDepth = boost::bind(&Renderer::renderDepth, renderer, _1);
		renderCloud = boost::bind(&Renderer::renderCloud, renderer, _1);
		renderCloudRGB = boost::bind(&Renderer::renderCloudRGB, renderer, _1);
		
		// Connect callback functions
		boost::signals2::connection c = grabber->registerCallback(I_CB);
		boost::signals2::connection c2 = grabber->registerCallback(D_CB);
		//toggleConnection(renderRGBConnection, renderRGB);
		toggleConnection(renderCloudRGBConnection, renderCloudRGB);

		// Initialize classifier
		std::vector<std::string> paths;
		//paths.push_back(".\\dataset\\.....");
		//NBNNClassifier->loadTrainingData(paths);

		// Callback for sending frame to classification thread
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> inpCB
			= boost::bind(&DataDistributor::classifierThreadInput, this, _1);
		grabber->registerCallback(inpCB);
	}
	
	~DataDistributor()
	{
		delete renderer;
		delete grabber;
	}

	void saveFrameCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (timeToSave)
		{
			if ( saveCloud(cloud) && saveRGB(*cloudToRGB(cloud)) )
				cout << "Saved " << numSaves << endl;
			timeToSave = false;
			numSaves++;
		}
		
		//processInput(cvWaitKey(1));
	}

	void classifierThreadInput(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		classificationInputMutex.lock();
		s_cloud = cloud;
		classificationInputMutex.unlock();

		processInput(cvWaitKey(1));

		classificationOutputMutex.lock();
		if( s_segmentedCloud ) {
			cout << "rendering segmented cloud" << endl;

			renderer->renderCloudRGB(s_segmentedCloud);
		}
		classificationOutputMutex.unlock();
	}

	// These are required for handling events in each thread
	void imageCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB){ processInput( cvWaitKey(1) ); }
	void depthCB(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth) { processInput( cvWaitKey(1) ); }
	void imageDepthCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB, const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth, float constant) { /*processInput( cvWaitKey(1) );*/ }
	void cloudCB(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {}

	template <class T>
	void toggleConnection(boost::signals2::connection& con, boost::function<T>& callback) {
		if (con.connected())
			con.disconnect();
		else
			con = grabber->registerCallback(callback);
	}

	bool saveCloud( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud )
	{
		std::stringstream depth_filename;
		depth_filename << saveFolder << "depth_" << numSaves << ".pcd";
		return pcl::io::savePCDFile(depth_filename.str(), *cloud.get(), true) == 0;
	}

	bool saveRGB( const cv::Mat& rgbImage)
	{
		std::stringstream  rgb_filename;
		rgb_filename << saveFolder << "rgb_" << numSaves << ".bmp";
		if(!cv::imwrite(rgb_filename.str(), rgbImage)) {
			cout << "Could not save: " << rgb_filename.str().c_str();
			return false;
		}
		return true;
	}

	boost::shared_ptr<cv::Mat> cloudToRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		boost::shared_ptr<cv::Mat> rgbImage(new cv::Mat(cloud->height, cloud->width, CV_8UC3));
		for (unsigned int x = 0; x < cloud->width; x++)
		{
			for (unsigned int y = 0; y < cloud->height; y++)
			{
				rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 2] = (*cloud)(x, y).r;
				rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 1] = (*cloud)(x, y).g;
				rgbImage->data[rgbImage->step[0]*y + rgbImage->step[1]*x + 0] = (*cloud)(x, y).b;
			}
		}
		return rgbImage;
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
			toggleConnection(renderRGBConnection, renderRGB);
			break;
		case '2':
			toggleConnection(renderDepthConnection, renderDepth);
			break;
		case '3':
			toggleConnection(renderCloudConnection, renderCloud);
			break;
		case '4':
			toggleConnection(renderCloudRGBConnection, renderCloudRGB);
			break;
		case 'r':
			cout << "Record mode on. Enter frame number to use as filename (e.g. rgb_FRAMENUM.bmp)" << endl;
			cin >> numSaves;
			grabber->registerCallback( saveCB );
			break;
		case 'b':
			cout << "update set" << endl;
			toggleConnection(renderCloudRGBConnection, renderCloudRGB);
			classificationInputMutex.lock();
			s_backgroundCloud = s_cloud;
			classificationInputMutex.unlock();
			break;
		default:
			return;
		}
	}
	
	void run ()
	{
		// Start classification thread
		boost::thread classify( ClassificationThread( classificationInputMutex, s_cloud, s_backgroundCloud,
													  classificationOutputMutex, s_segmentedCloud) );

		// start receiving point clouds
		grabber->start();

		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while (true)
			sleep(1);
			//processInput(cvWaitKey(33));

		// stop the grabber
		grabber->stop();
	}

protected:
	
	// Settings
	std::string saveFolder;
	int numSaves;
	bool timeToSave;
	
	// System components
	pcl::Grabber* grabber;
	Renderer* renderer;
	NBNN *NBNNClassifier;
	
	// Threading
	boost::mutex classificationInputMutex; // Locks all following:
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_backgroundCloud;

	boost::mutex classificationOutputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_segmentedCloud;

	// Callbacks
	boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> renderRGB;
	boost::function<void (const boost::shared_ptr<openni_wrapper::DepthImage>&)> renderDepth;
	boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)> renderCloud;
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)> renderCloudRGB;

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> saveCB;

	// Callback connections
	boost::signals2::connection renderRGBConnection;
	boost::signals2::connection renderDepthConnection;
	boost::signals2::connection renderCloudConnection;
	boost::signals2::connection renderCloudRGBConnection;
};

int main ()
{
	DataDistributor d;
	d.run();
	return 0;
}

/*
int
	main (int argc, char** argv)
{
	SegmentCloud SC;

	std::string path_back;
	std::cout << "Enter background pcd file path" << std::endl;
	std::cin >> path_back;
	// path_back = "saves/Background/depth_0.pcd";

	std::string path_img;
	std::cout << "Enter image pcd file path" << std::endl;
	std::cin >> path_img;
	// path_img = "saves/Pepper/depth_2.pcd";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_back, *cloud_back);
	SC.setBackgroundImage(cloud_back);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_img (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_img, *cloud_img);
	SC.setInputCloud(cloud_img);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentCloud;
	pcl::RangeImage rangeImage;

	std::cout << "Starting " << cloud_img->width << " " << cloud_img->height <<  std::endl;

	time_t start = time(NULL);
	// segmentCloud = SC.getNaNCloud();
	SC.getNaNCloud();
	
	time_t end = time(NULL);

	cvNamedWindow("TestMask", CV_WINDOW_AUTOSIZE); 
	cv::imshow("TestMask", *SC.BooleanMask);
	cv::waitKey();
	cvDestroyWindow("TestMask");

	std::cout << "Time: " << difftime(end, start) << std::endl;
	std::cout << "Saving "  <<  std::endl;


	pcl::io::savePCDFileASCII ("Saves/test_pcd.pcd", *cloud_img);
	std::cerr << "Saved " << cloud_img->points.size () << " data points to Saves/test_pcd.pcd." << std::endl;


	return (0);


}
*/