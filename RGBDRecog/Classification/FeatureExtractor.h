#pragma once

// also using opencv functions, which should be included in
// stdafx.h
#include <string>
#include "FeatureVector.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "FeatureData.h"
using namespace std;


//#define DENSE_SAMPLING

//Class used to calculate features from a given image.
//run loadCodebooks() first, then run either extractFeatures or extractRawFeatures
//for the codebooked or raw descriptor features
//also contains the names/info of all possible features
class FeatureExtractor
{
public:
	FeatureExtractor(void);
	~FeatureExtractor(void);

	// returns the name of the number'th features
	// crashes if feature number does not exist, so check with below function
	inline string getFeatureName(int number){return featureData->featureNames[number];};

	// get the amount of total possible features
	inline int getAmountOfFeatures(){return featureData->amountOfFeatures;};

	// check if the codebooks are loaded or not, you do not usually have to reload if
	// the codebooks are loaded
	inline bool codeBooksLoaded(){return codebooksloaded;};

	// call this if you want to use the codebooks (for extractFeatures)
	// check if this is loaded with the above function
	void loadCodebooks();
	void loadCodebooks(string filepath);

	// gets all specified descriptors from an image, and cv::Matches them with the available codebooks
	// the modes bool vector should be 'amountOfFeatures' long, with true for using the feature,
	// and false otherwise
	// call only after loadCodebooks()
	// mask is an optional parameter (initialized as an empty mat otherwise), that specifies
	// the region to extract the features from. 1 in the matrix means the pixel is used,
	// 0 if it isn't
	cv::Mat extractFeatures(vector<bool> modes, cv::Mat rgbimg, const cv::Mat mask  = cv::Mat());
	cv::Mat extractFeatures(vector<bool> modes, cv::Mat rgbimg, cv::Rect roi);

	// extract a vector of opencv cv::Matrixes with all raw descriptors
	// each cv::Mat in the vector contains in order the descriptors for SIFT/SURF or
	// other descriptions. Uses the same modes string as the above feature
	// Features are in the rows
	// call only after loadCodebooks()
	// for mask, see extractFeatures description
	// also comes with ROI version, that accept a cv::Rect
	vector<cv::Mat> extractRawFeatures(vector<bool> modes, cv::Mat rgbimg, const cv::Mat mask  = cv::Mat());
	vector<cv::Mat> extractRawFeatures(vector<bool> modes, cv::Mat rgbimg, cv::Rect roi);

	// Input the matrix dimensions (rows, cols), and a rectangle structure
	// with x and width the row directions, and y and height the column directions
	// Outputs a boolean mask to be used with all functions
	cv::Mat createMask(int rows, int cols, cv::Rect box);


	// converts a pointcloud containing RGB info to a cv::Mat image.
	static boost::shared_ptr<cv::Mat> FeatureExtractor::cloudToRGB(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

private:
	cv::SiftDescriptorExtractor * SDE; //used for extracting the features
	cv::SurfDescriptorExtractor * SuDE; //with the opponent color descriptors
	cv::OpponentColorDescriptorExtractor * des;
	cv::OpponentColorDescriptorExtractor * desSURF;

	FeatureData * featureData;

	cv::SIFT * siftdetector; //two opencv detector algorithms
	cv::SURF * surfdetector;

	FeatureVector* codebooks;

	// these functions are used to calculate their respective descriptors
	// These are called by the two extractFeatures functions above
	// use those as an interface
	cv::Mat normalSift(const cv::Mat grayimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	cv::Mat hueSift(const cv::Mat grayimg, const cv::Mat hueimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	cv::Mat opSift(const cv::Mat grayimg, const cv::Mat rgbimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	cv::Mat normalSurf(const cv::Mat grayimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	cv::Mat hueSurf(const cv::Mat grayimg, const cv::Mat hueimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	cv::Mat opSurf(const cv::Mat grayimg, const cv::Mat rgbimg, const cv::Mat mask, vector<cv::KeyPoint> keypoints =  vector<cv::KeyPoint>());
	
	bool codebooksloaded; //are the codebooks loaded or not?

	//helper function that adds a descriptor to the tempfeaturevector
	void addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector,cv::Mat & descriptors, int mode);
};

