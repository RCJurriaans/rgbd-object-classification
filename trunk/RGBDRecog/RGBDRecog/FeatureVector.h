#pragma once
#include <cv.h>
using namespace cv;

class FeatureVector
{
public:
	FeatureVector(void);
	~FeatureVector(void);
	Mat* features;
	int numFeatures;

	void AddFeatures(Mat keypoints);
	Mat* kmeans(int dicsize);
private:
	
};

