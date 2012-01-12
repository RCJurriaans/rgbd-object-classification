#pragma once
#include <cv.h>
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

	void AddFeatures(Mat keypoints); //add keypoints to the features, used in the codebook creation
	Mat* kmeans(int dicsize); //apply kmeans on the features in the class // note: this is very slow :(
private:
	
};

