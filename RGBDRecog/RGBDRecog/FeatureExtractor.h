#pragma once
#include <string>
//#include <cv.h>
#include "FeatureVector.h"
using namespace std;
//using namespace cv;

class FeatureExtractor
{
public:
	FeatureExtractor(void);
	~FeatureExtractor(void);

	
	inline string getFeatureName(int number){return featureNames[number];};
	inline int getAmountOfFeatures(){return amountOfFeatures;};

	inline bool codeBooksLoaded(){return codebooksloaded;};

	//call this if you want to use the codebooks (for extractFeatures)
	void loadCodebooks();

	//gets all specified descriptors from an image, and cv::Matches them with the available codebooks
	cv::Mat extractFeatures(vector<bool> modes, cv::Mat rgbimg);

	//extract a vector of opencv cv::Matrixes with all raw descriptors
	//each cv::Mat in the vector contains in order the descriptors for SIFT/SURF or
	//other descriptions.
	//Features are in the rows!
	vector<cv::Mat> extractRawFeatures(vector<bool> modes, cv::Mat rgbimg);
private:
	cv::SiftDescriptorExtractor * SDE;
	cv::SurfDescriptorExtractor * SuDE;
	cv::OpponentColorDescriptorExtractor * des;
	cv::OpponentColorDescriptorExtractor * desSURF;
	string* featureNames;
	int amountOfFeatures;

	cv::SIFT * siftdetector;
	cv::SURF * surfdetector;

	FeatureVector* codebooks;

	cv::Mat normalSift(const cv::Mat grayimg);
	cv::Mat hueSift(const cv::Mat grayimg,const  cv::Mat hueimg);
	cv::Mat opSift(const cv::Mat grayimg,const  cv::Mat rgbimg);
	cv::Mat normalSurf(const cv::Mat grayimg);
	cv::Mat hueSurf(const cv::Mat grayimg,const  cv::Mat hueimg);
	cv::Mat opSurf(const cv::Mat grayimg,const  cv::Mat rgbimg);
	
	bool codebooksloaded;

	void addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector,cv::Mat & descriptors, int mode);
};

