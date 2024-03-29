// DataDistributor.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "FeatureExtractor.h"
#include "Renderer.h"
#include "ClassificationThread.h"
#include "ClassificationResults.h"
#include "RenderThread.h"

#include <boost\filesystem\convenience.hpp>

//using namespace std;

#ifdef WIN32
# define sleep(x) Sleep((x)*1000) 
#endif

string convertNumberToFLString(int length, int number){
	int tempnum = number;
	string returnvalue;
	
	length--;
	
		while(tempnum/10 != 0){
			tempnum = tempnum/10;
			length--;
		}
		for(int k = 0; k < length; k++){
			returnvalue += "0";
		}
		returnvalue += boost::lexical_cast<string>(number);
	return returnvalue;
}


class DataDistributor
{
public:
	DataDistributor() :
		grabber(new pcl::OpenNIGrabber()),
		saveFolder(".\\saves\\"),
		numSaves(0),
		timeToSave(false)
		,currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
		,results(new ClassificationResults())
		,s_newClassName("NewClassName")
		,s_makeNewClass(false)
		,s_trainNow(false)
		,s_addDataPointNow(false)
	{
		renderer = new Renderer(boost::bind(&DataDistributor::processInput, this, _1), results);
		lastSave = time(NULL);
		automode = false;

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
		//boost::signals2::connection c2 = grabber->registerCallback(D_CB);
		//boost::signals2::connection c3 = grabber->registerCallback(C_CB);
		
		//toggleConnection(renderRGBConnection, renderRGB);
		//toggleConnection(renderCloudConnection, renderCloud);
		//toggleConnection(renderCloudRGBConnection, renderCloudRGB);

		// Callback for sending frame to classification thread
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &)> inpCB
			= boost::bind(&DataDistributor::distributeCloudRGB, this, _1);
		boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &)> inpCB2
			= boost::bind(&DataDistributor::distributeCloud, this, _1);
		grabber->registerCallback(inpCB);
	}
	
	~DataDistributor()
	{
		delete renderer;
		delete grabber;
	}

	void distributeCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
	{
//		cout << "rendering cloud"<<endl;
//		renderer->renderCloud(cloud);
	}

	void distributeCloudRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		currentCloud = cloud;

		//cout << "Sending cloud to classification thread"<<endl;
		classificationInputMutex.lock();
		s_cloud = cloud;
		classificationInputMutex.unlock();

		//processInput(cvWaitKey(1));

		/*classificationOutputMutex.lock();
		
		if( s_segmentedCloud ) {
			//cout << "rendering segmented cloud" << endl;
			//renderer->ROI = s_ROI;
			cout << "main ROI: " << s_ROI.x << " " << s_ROI.y << " " << s_ROI.width << " " << s_ROI.height << endl;	
			renderer->setResults(s_ROI);
			//renderer->renderCloudRGB(s_segmentedCloud, s_ROI);
		}
		classificationOutputMutex.unlock();
		*/
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

	// Saving utilities
	bool saveCloud( const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, std::string name = "img" )
	{
		std::stringstream depth_filename;
		depth_filename << saveFolder << "\\" << name << convertNumberToFLString(3, numSaves) << ".pcd";
		return pcl::io::savePCDFile(depth_filename.str(), *cloud.get(), true) == 0;
	}

	bool saveRGB( const cv::Mat& rgbImage, std::string name = "img")
	{
		std::stringstream  rgb_filename;
		rgb_filename << saveFolder << "\\" << name << convertNumberToFLString(3, numSaves) << ".bmp";
		return cv::imwrite(rgb_filename.str(), rgbImage);
	}

	void saveFrameCB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if( automode ) {
 			time_t currTime = time( NULL );
			if( difftime(currTime, lastSave) > 2.0 ) {
				timeToSave = true;
				lastSave = time(NULL);
			}
		}

		if (timeToSave)
		{
			if ( saveCloud(cloud) /*&& saveCloud(backgroundCloud, "bg") */&& saveRGB(*FeatureExtractor::cloudToRGB(cloud))) {
				cout << "Saved " << numSaves << endl;
				numSaves++;
			}
			else {
				cout << "Could not save!" << endl; 
			}
			timeToSave = false;
		}
	}

	void processInput( int key )
	{
		int onlineLearnNum = -1;
		string onlineLearnName;
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
		
		
			/*
			case 'b':
			cout << "Setting background" << endl;
			classificationInputMutex.lock();
			s_backgroundCloud = currentCloud;
			backgroundCloud = currentCloud;
			classificationInputMutex.unlock();
			break;
		case 's':
			cout << "Save mode on." << endl;
			grabber->registerCallback( saveCB );
			break;
		case 'f':
			cout << "Enter save folder" << endl;
			cin >> saveFolder;

			//if (!boost::filesystem::exists( saveFolder ))
			//	cout << "EPIC FAIL: INVALID FOLDER, please try again or press esc to give up."<<endl;
			//else {
				cout << "New save folder: " << saveFolder << endl;
				cout << "Starting saves at one.." << endl;
				numSaves = 1;
			//}
			break;
		case 'n':
			cout << "Enter save number" << endl;
			cin >> numSaves;
			break;
		case 'a':
			automode = automode == false;
			cout << "Toggling auto-save-mode to: "<< automode <<endl;
			break;*/

		// Online learning
		case 'n':
			cout << "Enter name of class" << endl;
			cin >> onlineLearnName;
			classificationInputMutex.lock();
			//s_newClassName = onlineLearnName;
			s_makeNewClass = true;
			classificationInputMutex.unlock();
			
			results->mtx.lock();
				results->newClassName = onlineLearnName;
			results->mtx.unlock();
			break;
		case 'a':
			classificationInputMutex.lock();
			cout << "Adding new datapoint to class (signal from main)" << endl;
			s_addDataPointNow = true;
			classificationInputMutex.unlock();
			break;
		case 't':
			cout << "Training on new data" << endl;
			classificationInputMutex.lock();
			s_trainNow = true;
			classificationInputMutex.unlock();
			break;

		// What to render
		case 'p':
			results->mtx.lock();
				results->renderPlane = results->renderPlane == false;
			results->mtx.unlock();
			cout << "Plane mode: " << results->renderPlane << endl;
			break;
		case 'b':
			results->mtx.lock();
				results->renderBox = results->renderBox == false;
			results->mtx.unlock();
			cout << "Bounding box mode: " << results->renderBox << endl;
			break;
		case 'c':
			results->mtx.lock();
				results->classify = results->classify == false;
			results->mtx.unlock();
			cout << "Classification mode: " << results->classify << endl;
			break;
		default:
			return;
		}
	}
	
	void run ()
	{
		// Start classification thread
		ClassificationThread classificationThread( classificationInputMutex, s_cloud, s_backgroundCloud,
												    results,
													//s_newClassName,
													s_makeNewClass,
													s_trainNow,
													s_addDataPointNow );
		boost::thread classify( &ClassificationThread::run, &classificationThread  );
		
		RenderThread renderObject( results );
		boost::thread renderThread( &RenderThread::run, &renderObject );

		// start receiving point clouds
		grabber->start();

		// wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
		while (true)
			sleep(1);
			processInput(cvWaitKey(1));

		// stop the grabber
		grabber->stop();
	}

protected:
	
	// Settings
	std::string saveFolder;
	int numSaves;
	bool timeToSave;
	time_t lastSave;
	bool automode;

	// System components
	pcl::Grabber* grabber;
	Renderer* renderer;

	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr currentCloud;
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr backgroundCloud;

	// Threading
	boost::mutex classificationInputMutex; // Locks all following:
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_cloud;
	  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr s_backgroundCloud;
	  bool s_addDataPointNow;
	  string s_newClassName;
	  bool s_trainNow;
	  bool s_makeNewClass;

	//boost::mutex visualizationInputMutex;
	boost::shared_ptr<ClassificationResults> results;
	
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

//#define SEG

#ifndef SEG
int main ()
{
	DataDistributor d;
	d.run();
	return 0;
}
#else
int	main (int argc, char** argv)
{
	SegmentCloud::tester();

	return (0);


}
#endif