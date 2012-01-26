


#include "stdafx.h"

#include "NBNN.h"
#include <opencv2\ml\ml.hpp>

using namespace std;

bool NBNN::loadTrainingData(std::string filename, std::string dataname, int numClasses)
{
	CvFileStorage *dataset= cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ ) ;

	for (unsigned int i = 0; i < numClasses; i++){
		stringstream nodeName;
		nodeName << dataname << i;
		CvFileNode* node = cvGetFileNodeByName(dataset, 0, nodeName.str().c_str());
		
	}


	//std::numeric_limits<float>::q
/*	for (vector<string>::iterator descPath = classDescriptorPaths.begin(); descPath != classDescriptorPaths.end(); ++descPath)
	{
		// Type of the file is determined from the content
		cv::FileStorage fs(*descPath, cv::FileStorage::READ);
		
		//cv::Mat M;
		//fs["descriptors"] >> M;
		vector<cv::Mat> fileDescriptors;
		fs["descriptors"] >> fileDescriptors;
		
		addInstancesToClass(fileDescriptors);
	}*/
	return true;
}

unsigned int NBNN::addInstancesToClass(const vector<cv::Mat>& descriptors, int classNum)
{
	if (classNum == -1) // Make a new class
	{
		classMatchers[numClasses] = new cv::FlannBasedMatcher();
		classNum = numClasses;
		numClasses++;
	}
	
	classMatchers[classNum]->add(descriptors);
	//classMatchers[classNum]->train();

	return classNum;
}

unsigned int NBNN::classify(const cv::Mat& queryDescriptors)
{
	vector<double> classScores; // Might be useful to return these at some point.
	double minScore = 99999999999999999.0;
	int minClass = -1;
	for (unsigned int C = 0; C < numClasses; C++)
	{
		std::vector< cv::DMatch > matches;
		
		classMatchers[C]->match(queryDescriptors, matches);

		classScores.push_back(0);
		for(vector<cv::DMatch>::iterator it = matches.begin(); it != matches.end(); ++it) {
			classScores[C] += it->distance * it->distance;
		}

		if (classScores[C] < minScore)
		{
			minScore = classScores[C];
			minClass = C;
		}
	}

	return minClass;
}
