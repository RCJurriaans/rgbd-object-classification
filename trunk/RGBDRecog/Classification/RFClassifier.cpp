#include "StdAfx.h"
#include "RFClassifier.h"

//#include <cv.h>
//#include "ml.h"
//using namespace cv;
using namespace std;

RFClassifier::RFClassifier(void)
{
	treestructure = new CvRTrees();
}

RFClassifier::RFClassifier(string filename, string dataname)
{
	RFClassifier();
	read(filename, dataname);
}

RFClassifier::~RFClassifier(void)
{
	//delete treestructure;
}

int RFClassifier::predict(cv::Mat input){
	return static_cast<int>(treestructure->predict(input,cv::Mat()));
}

void RFClassifier::trainTree(vector<cv::Mat> trainingData){
	
	//create a cv::Matrix with all the trainingData merged
	//this is needed as input for the random tree trainer
	cv::Mat mergedData;
	mergedData = trainingData[0];
	for(unsigned int i = 1; i < trainingData.size(); i++){
		hconcat(mergedData,trainingData[i],mergedData);
	}

	//create a response cv::Matrix with different numbers for each class
	cv::Mat responses(mergedData.cols,1,mergedData.type());
	for(unsigned int i = 0; i < trainingData.size();i++){
		int multifactor=0;
		for(unsigned int j = 0; j < i; j++){
			multifactor += trainingData[j].cols; //fast counter for how many previous points we added
		}
		for(int j = 0; j < trainingData[i].cols; j++){
			responses.at<float>(j+multifactor) = static_cast<float>(i);
		}
	}
	treestructure->train(mergedData,CV_COL_SAMPLE,responses,cv::Mat(),cv::Mat(),cv::Mat(),cv::Mat(),CvRTParams());
}

void RFClassifier::write(string filename, string dataname){

	cout << "Saving tree to: " << filename << endl;


	CvFileStorage * output= cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_WRITE_TEXT ) ;
	treestructure->write(output, dataname.c_str());

	cvReleaseFileStorage(&output);
}

void RFClassifier::read(string filename, string dataname){
	CvFileStorage * input= cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ ) ;
	cout << "Reading tree from: " << filename << endl;
//	treestructure->read(input, cvGetFileNodeByName(input,0,dataname.c_str()));
	CvFileNode* node = cvGetFileNodeByName(input,0,dataname.c_str());
	treestructure->read(input, node);
	cvReleaseFileStorage(&input);
}