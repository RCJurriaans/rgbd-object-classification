


#include "stdafx.h"

#include "NBNN.h"
#include <opencv2\ml\ml.hpp>
#include <limits>

using namespace std;

void NBNN::write( cv::FileStorage& f )
{
	f << "numClasses" << (int)numClasses;
	//f << "classes" << "[";
	for(unsigned int i = 0; i < numClasses; i++)
	{
		stringstream classDataName;
		classDataName << "class" << i;
		//f << classDataName.str() << classMatchers[i]->getTrainDescriptors() ;
		f << classDataName.str() <<  "[" << classMatchers[i]->getTrainDescriptors() << "]";
		//for (int j = 0; j < classMatchers[i]->getTrainDescriptors().size(); j++)
		//	f << classMatchers[i]->getTrainDescriptors().at(j);
//		f << "]";
	}
	//f << "]";
}

void NBNN::read( const cv::FileStorage& f )
{
	int nc;
	f["numClasses"] >> nc;
	cout << "numClasses: " << nc;

	//cv::FileNode classes = f["classes"];
//	CV_Assert(classes.type() == cv::FileNode::SEQ && classes.size() == numClasses);
	vector<cv::Mat> classInstances;
	for(unsigned int i = 0; i < nc; i++)
	{
		classInstances.clear();
		
		stringstream classDataName;
		classDataName << "class" << (int)i;
		f[classDataName.str()] >> classInstances;

		//int a;
		//f["class3"] >> classInstances;
		//cout << "aaa" << a;
		addInstancesToClass(classInstances);
	}
}


unsigned int NBNN::addInstancesToClass(const vector<cv::Mat>& descriptors, int classNum)
{
	if (classNum == -1) // Make a new class
	{
		classMatchers.push_back(new cv::FlannBasedMatcher());
		classNum = numClasses;
		numClasses++;
	}
	
	classMatchers[classNum]->add(descriptors);
	classMatchers[classNum]->train();
	return classNum;
}

unsigned int NBNN::classify(const cv::Mat& queryDescriptors)
{
	vector<double> classScores; // Might be useful to return these at some point.
	double minScore = numeric_limits<double>::max( );	// Distance of closest class found
	int minClass = -1;	// Id of closest class
	for (unsigned int C = 0; C < numClasses; C++)
	{
		// Find matches for each 
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
