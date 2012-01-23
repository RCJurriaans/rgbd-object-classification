#pragma once

//#include "ml.h"
#include <opencv2\ml\ml.hpp>
#include <string>
using namespace std;

class RFClassifier
{
public:
	RFClassifier(void);
	RFClassifier(string filename, string dataname);
	~RFClassifier(void);

	//takes an input cv::Mat, and classifies it with the random forest
	//be sure that the amount of input parameters is the same
	int predict(cv::Mat input);

	// takes a trainingData cv::Matrix, with for each cv::Matrix in the vector 
	// containing the inforation for each class
	// The predicted numbers will be integer numbers, that depend on the 
	// ordering of the classes in the trainingData
	// the amount of columns is the amount of datapoints
	void trainTree(vector<cv::Mat> trainingData);
	
	//writing data to a file named randomforest + modestring + .yml
	void write(string filename, string dataname);

	//read a tree with this modestring specifier
	void read(string filename, string dataname);
private:
	CvRTrees* treestructure; // the tree that is used :)
};

