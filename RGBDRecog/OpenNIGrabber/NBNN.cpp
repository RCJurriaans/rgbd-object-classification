


#include "stdafx.h"
/*
#include "NBNN.h"
#include <ml.h>

using namespace std;

bool NBNN::loadTrainingData(vector<string> classDescriptorPaths)
{
	//std::numeric_limits<float>::q
	for (vector<string>::iterator descPath = classDescriptorPaths.begin(); descPath != classDescriptorPaths.end(); ++descPath)
	{
		// Type of the file is determined from the content
		cv::FileStorage fs(*descPath, cv::FileStorage::READ);
		
		//cv::Mat M;
		//fs["descriptors"] >> M;
		vector<cv::Mat> fileDescriptors;
		fs["descriptors"] >> fileDescriptors;
		
		addInstancesToClass(fileDescriptors);
	}
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
	double minScore = 99999999999999999;
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


unsigned int NBNN::relationalClassify(const cv::Mat& queryDescriptors)
{
	vector<double> classScores; // Might be useful to return these at some point.
	double minScore = 99999999999999999;
	int minClass = -1;
	for (unsigned int C = 0; C < numClasses; C++)
	{
		std::vector< cv::DMatch > matches;
		
		classMatchers[C]->match(queryDescriptors, matches);

		classScores.push_back(0);
		for(vector<cv::DMatch>::iterator it = matches.begin(); it != matches.end(); ++it) {
			classScores[C] += it->distance * it->distance;
		}

		// Loop over descriptor pairs, and see how much the relation matches the relation between the NN pair.


		if (classScores[C] < minScore)
		{
			minScore = classScores[C];
			minClass = C;
		}
	}

	return minClass;
}

*/