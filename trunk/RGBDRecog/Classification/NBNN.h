
#pragma once
#include <vector>
#include "opencv2\features2d\features2d.hpp"

class NBNN //: public Classifier
{
public:

	NBNN() :
	  numClasses(0)
	{}

	NBNN(std::vector<std::string> classDescriptorPaths) :
	  numClasses(0)
	{
		loadTrainingData(classDescriptorPaths);
	}

	bool loadTrainingData(std::vector<std::string> classDescriptorPaths);
	unsigned int addInstancesToClass(const std::vector<cv::Mat>& descriptors, int classNum = -1);
	unsigned int classify(const cv::Mat& queryDescriptors);

protected:
	unsigned int numClasses;
	std::vector<cv::FlannBasedMatcher*> classMatchers;

private:


};
