#pragma once

// also using opencv functions, which should be included in
// stdafx.h
#include <string>
#include "FeatureVector.h"
using namespace std;


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
	inline string getFeatureName(int number){return featureNames[number];};

	// get the amount of total possible features
	inline int getAmountOfFeatures(){return amountOfFeatures;};

	// check if the codebooks are loaded or not, you do not usually have to reload if
	// the codebooks are loaded
	inline bool codeBooksLoaded(){return codebooksloaded;};

	// call this if you want to use the codebooks (for extractFeatures)
	// check if this is loaded with the above function
	void loadCodebooks();

	// gets all specified descriptors from an image, and cv::Matches them with the available codebooks
	// the modes bool vector should be 'amountOfFeatures' long, with true for using the feature,
	// and false otherwise
	// call only after loadCodebooks(); !
	cv::Mat extractFeatures(vector<bool> modes, cv::Mat rgbimg);

	//extract a vector of opencv cv::Matrixes with all raw descriptors
	//each cv::Mat in the vector contains in order the descriptors for SIFT/SURF or
	//other descriptions. Uses the same modes string as the above feature
	//Features are in the rows
	// call only after loadCodebooks(); !
	vector<cv::Mat> extractRawFeatures(vector<bool> modes, cv::Mat rgbimg);
private:
	cv::SiftDescriptorExtractor * SDE; //used for extracting the features
	cv::SurfDescriptorExtractor * SuDE; //with the opponent color descriptors
	cv::OpponentColorDescriptorExtractor * des;
	cv::OpponentColorDescriptorExtractor * desSURF;
	string* featureNames; //contains the names of all features
	int amountOfFeatures; //the amount of total possible features that can be used

	cv::SIFT * siftdetector; //two opencv detector algorithms
	cv::SURF * surfdetector;

	FeatureVector* codebooks;

	// these functions are used to calculate their respective descriptors
	// These are called by the two extractFeatures functions above
	// use those as an interface
	cv::Mat normalSift(const cv::Mat grayimg);
	cv::Mat hueSift(const cv::Mat grayimg,const  cv::Mat hueimg);
	cv::Mat opSift(const cv::Mat grayimg,const  cv::Mat rgbimg);
	cv::Mat normalSurf(const cv::Mat grayimg);
	cv::Mat hueSurf(const cv::Mat grayimg,const  cv::Mat hueimg);
	cv::Mat opSurf(const cv::Mat grayimg,const  cv::Mat rgbimg);
	
	bool codebooksloaded; //are the codebooks loaded or not?

	//helper function that adds a descriptor to the tempfeaturevector
	void addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector,cv::Mat & descriptors, int mode);
};

