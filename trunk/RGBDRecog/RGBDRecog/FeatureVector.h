#pragma once
#include <cv.h>
#include <ml.h>
#include <highgui.h>
using namespace cv;

//Implements a opencv Matrix that contains features in the rows
//can be used to store and perform most calculations on
//like saving and loading codebooks etc.
class FeatureVector
{
public:
	FeatureVector(void);
	~FeatureVector(void);
	Mat* features;

	void AddFeatures(Mat & descriptorIn); //add keypoints to the features, used in the codebook creation
	Mat* kmeans(int dicsize); //apply kmeans on the features in the class // note: this is very slow :(
	Mat GenHistogram(const Mat descriptors); // take a set of feature descriptors, and create a codebook histogram of these
	CvKNearest* knn;
	void TrainkNN(); 
private:
	
};

