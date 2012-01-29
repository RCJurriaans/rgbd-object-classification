
#pragma once
#include <vector>
#include "opencv2\features2d\features2d.hpp"

class NBNN
{
public:

	NBNN() :
	  numClasses(0)
	{}

	NBNN(std::vector<std::string> classDescriptorPaths) :
	  numClasses(0)
	{
//		loadTrainingData(classDescriptorPaths);
	}

	//bool NBNN::loadTrainingData(std::string filename, std::string dataname, int numClasses);
	unsigned int addInstancesToClass(const std::vector<cv::Mat>& descriptors, int classNum = -1);
	unsigned int classify(const cv::Mat& queryDescriptors);

	void write( cv::FileStorage& f );
	void read( const cv::FileStorage& f );

protected:
	unsigned int numClasses;
	std::vector<cv::FlannBasedMatcher*> classMatchers;

private:


};
