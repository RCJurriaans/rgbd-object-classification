// DataDistributor.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "RenderThread.h"
#include "FeatureExtractor.h"
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
		,currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
		//,renderThread(new RenderThread())
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

		//renderRGB = boost::bind(&Renderer::renderRGB, renderer, _1);
		//renderDepth = boost::bind(&Renderer::renderDepth, renderer, _1);
		//renderCloud = boost::bind(&Renderer::renderCloud, renderer, _1);
		//renderCloudRGB = boost::bind(&Renderer::renderCloudRGB, renderer, _1);
		
		// Connect callback functions
		boost::signals2::connection c = grabber->registerCallback(I_CB);
		boost::signals2::connection c2 = grabber->registerCallback(D_CB);
		boost::signals2::connection c3 = grabber->registerCallback(C_CB);
		
		//toggleConnection(renderRGBConnection, renderRGB);
		//toggleConnection(renderCloudConnection, renderCloud);
		//toggleConnection(renderCloudRGBConnection, renderCloudRGB);

		// Initialize classifier
		//std::vector<std::string> paths;
		//paths.push_back(".\\dataset\\.....");
		//NBNNClassifier->loadTrainingData(paths);

		// Callback for sending frame to classification thread
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> inpCB
			= boost::bind(&DataDistributor::distributeCloudRGB, this, _1);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)> inpCB2
			= boost::bind(&DataDistributor::distributeCloud, this, _1);
		grabber->registerCallback(inpCB);

		//cloudRenderer =  new CloudRenderer();
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
			if ( saveCloud(cloud) && saveRGB(*FeatureExtractor::cloudToRGB(cloud)) )
				cout << "Saved " << numSaves << endl;
			timeToSave = false;
			numSaves++;
		}
		
		//processInput(cvWaitKey(1));
	}
	
	void distributeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
		cout << "rendering cloud"<<endl;
		renderer->renderCloud(cloud);
	}

	void distributeCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		//renderer->renderCloudRGB(cloud);
		currentCloud = cloud;

		//cout << "Sending cloud to classification thread"<<endl;
		classificationInputMutex.lock();
		s_cloud = cloud;
		classificationInputMutex.unlock();

		//processInput(cvWaitKey(1));

		classificationOutputMutex.lock();
		if( s_segmentedCloud ) {
			//cout << "rendering segmented cloud" << endl;
			//renderer->ROI = s_ROI;
			renderer->renderCloudRGB(s_segmentedCloud, s_ROI);
		}
		classificationOutputMutex.unlock();

		//processInput(cvWaitKey(10));
	}

	// These are required for handling events in each thread
	void imageCB(const boost::shared_ptr<openni_wrapper::Image>& oniRGB){ renderer->renderRGB(oniRGB); processInput( cvWaitKey(1) ); }
	void depthCB(const boost::shared_ptr<openni_wrapper::DepthImage>& oniDepth) { renderer->renderDepth(oniDepth); processInput( cvWaitKey(1) ); }
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
		case 's':
			cout << "Save mode on. Enter frame number to use as filename (e.g. rgb_FRAMENUM.bmp)" << endl;
			cin >> numSaves;
			grabber->registerCallback( saveCB );
			break;
		case 'b':
			cout << "Setting background" << endl;
			classificationInputMutex.lock();
			s_backgroundCloud = currentCloud;//s_cloud;
			classificationInputMutex.unlock();
			break;
		default:
			return;
		}
	}
	
	void run ()
	{
		// Start classification thread
		ClassificationThread classificationThread( classificationInputMutex, s_cloud, s_backgroundCloud,
													  classificationOutputMutex, s_segmentedCloud);
		boost::thread classify( &ClassificationThread::run, &classificationThread  );
		
		//cout << "run" <<endl;
		//RenderThread r( visualizationInputMutex );
		//boost::thread visualize( &RenderThread::run, renderThread );

		// start receiving point clouds
		grabber->start();

		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while (true)
			sleep(1);
			processInput(cvWaitKey(33));

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

	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr currentCloud;

	// Threading
	boost::mutex classificationInputMutex; // Locks all following:
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_backgroundCloud;

	boost::mutex classificationOutputMutex;
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr s_segmentedCloud;
	  cv::Rect s_ROI;

	boost::mutex visualizationInputMutex;
	//RenderThread* renderThread;
	
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
/*
int main ()
{
	DataDistributor d;
	d.run();
	return 0;
}*/


int
	main (int argc, char** argv)
{
	SegmentCloud SC;

	std::string path_back;
	//std::cout << "Enter background pcd file path" << std::endl;
	//std::cin >> path_back;
	 path_back = "img001.pcd";

	std::string path_img;
	//std::cout << "Enter image pcd file path" << std::endl;
	//std::cin >> path_img;
	 path_img = "img002.pcd";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_back, *cloud_back);
//	SC.setBackgroundImage(cloud_back);
	std::cout << "Background loaded" << std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_img (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (path_img, *cloud_img);
	//SC.setInputCloud(cloud_img);
	std::cout << "Input loaded" << std::endl;


	std::cout << "Starting " << cloud_img->width << " " << cloud_img->height <<  std::endl;

	time_t start = time(NULL);
	
	//SC.getNaNCloud();
	//SC.getUnorgCloud();
	//SC.getROI();
	cloud_img = SC.getWindowCloud(cloud_img, cloud_back);

	time_t end = time(NULL);

	std::cout << "Ending " << cloud_img->width << " " << cloud_img->height <<  std::endl;

	/*
	int mean_x = SC.imcalc.getmean(0,0);
	int mean_y = SC.imcalc.getmean(0,1);

	int boxWidth  = SC.imcalc.getRegions(0,0);
	int boxHeight = SC.imcalc.getRegions(0,1);

	boxWidth *= 1.2;
	boxHeight *= 1.2;


	int xmin = mean_x-boxWidth/2;
	int xmax = mean_x+boxWidth/2;
	int ymin = mean_y-boxHeight/2;
	int ymax = mean_y+boxHeight/2;

	IplImage ipl_bmask = SC.BooleanMask;
	
	cvRectangle(&ipl_bmask,                  
                cvPoint(xmin, ymin),        
                cvPoint(xmax, ymax),       
                cvScalar(255,0,0)); 
                

	//cvNamedWindow("TestMask", CV_WINDOW_AUTOSIZE); 
	//cv::imshow("TestMask", SC.BooleanMask);
	cvShowImage("TestMask", &ipl_bmask);
	cv::waitKey();
	cvDestroyWindow("TestMask");

	

	std::cout << "Time: " << difftime(end, start) << std::endl;
	std::cout << "Saving "  <<  std::endl;
	*/

	pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud_img);
	std::cerr << "Saved " << cloud_img->points.size () << " data points to test_pcd.pcd." << std::endl;
	

	return (0);


}
