#pragma once

#include "StdAfx.h"

//Implements a opencv cv::Matrix that contains features in the rows
//can be used to store and perform most calculations on
//like saving and loading codebooks etc.
class FeatureVector
{
public:
	FeatureVector(void);
	~FeatureVector(void);
	cv::Mat* features;

	void AddFeatures(cv::Mat & descriptorIn); //add keypoints to the features, used in the codebook creation
	cv::Mat* kmeans(int dicsize); //apply kmeans on the features in the class // note: this is very slow :(
	cv::Mat GenHistogram(const cv::Mat descriptors); // take a set of feature descriptors, and create a codebook histogram of these
	CvKNearest* knn;
	void TrainkNN(); 
private:
	
};

