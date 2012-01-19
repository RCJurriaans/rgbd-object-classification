#pragma once
#include <string>
#include <cv.h>
#include "FeatureVector.h"
using namespace std;
using namespace cv;

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

	//gets all specified descriptors from an image, and matches them with the available codebooks
	cv::Mat extractFeatures(vector<bool> modes, cv::Mat rgbimg);

	//extract a vector of opencv matrixes with all raw descriptors
	//each Mat in the vector contains in order the descriptors for SIFT/SURF or
	//other descriptions.
	//Features are in the rows!
	vector<Mat> extractRawFeatures(vector<bool> modes, cv::Mat rgbimg);
private:
	SiftDescriptorExtractor * SDE;
	SurfDescriptorExtractor * SuDE;
	OpponentColorDescriptorExtractor * des;
	OpponentColorDescriptorExtractor * desSURF;
	string* featureNames;
	int amountOfFeatures;

	SIFT * siftdetector;
	SURF * surfdetector;

	FeatureVector* codebooks;

	Mat normalSift(const Mat grayimg);
	Mat hueSift(const Mat grayimg,const  Mat hueimg);
	Mat opSift(const Mat grayimg,const  Mat rgbimg);
	Mat normalSurf(const Mat grayimg);
	Mat hueSurf(const Mat grayimg,const  Mat hueimg);
	Mat opSurf(const Mat grayimg,const  Mat rgbimg);
	
	bool codebooksloaded;

	void addDescriptor(bool & firstadded, Mat & tempfeaturevector,Mat & descriptors, int mode);
};

