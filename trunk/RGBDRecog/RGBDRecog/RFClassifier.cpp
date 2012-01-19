#include "StdAfx.h"
#include "RFClassifier.h"

#include <cv.h>
#include "ml.h"
using namespace cv;
using namespace std;

RFClassifier::RFClassifier(void)
{
	treestructure = new CvRTrees();
}


RFClassifier::~RFClassifier(void)
{
	//delete treestructure;
}

float RFClassifier:: predict(Mat input){
	return treestructure->predict(input,Mat());
}

void RFClassifier:: trainTree(vector<Mat> trainingData){
	
	//create a matrix with all the trainingData merged
	//this is needed as input for the random tree trainer
	Mat mergedData;
	mergedData = trainingData[0];
	for(unsigned int i = 1; i < trainingData.size(); i++){
		hconcat(mergedData,trainingData[i],mergedData);
	}

	//create a response matrix with different numbers for each class
	Mat responses(mergedData.cols,1,mergedData.type());
	for(unsigned int i = 0; i < trainingData.size();i++){
		int multifactor=0;
		for(unsigned int j = 0; j < i; j++){
			multifactor += trainingData[j].cols; //fast counter for how many previous points we added
		}
		for(int j = 0; j < trainingData[i].cols; j++){
			responses.at<float>(j+multifactor) = static_cast<float>(i);
		}
	}
	treestructure->train(mergedData,CV_COL_SAMPLE,responses,Mat(),Mat(),Mat(),Mat(),CvRTParams());
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
	treestructure->read(cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ ), cvGetFileNodeByName(cvOpenFileStorage(filename.c_str(), 0, CV_STORAGE_READ ),0,dataname.c_str()));
	cvReleaseFileStorage(&input);
}