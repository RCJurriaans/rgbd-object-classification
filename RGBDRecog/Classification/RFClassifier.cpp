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

void RFClassifier::readTrainingData( string modestring )
{
	ifstream infile;
	string filename;
	filename += getenv("RGBDDATA_DIR") ;
	filename += "\\data_info.txt";
 	infile.open(filename); //open data_info.txt etc.
	if(!infile.good()){
		cout << "Could not find %RGBDATA_DIR%/data_info.txt" << endl;
		return;
	}
	
	char temps[256];
	infile.getline(temps,256);
	infile >> amountOfClasses; //amount of classes in dataset

	
	string trainPath =  getenv("RGBDDATA_DIR");
	trainPath += "\\trainingdata" + modestring + "RF.yml";
	cv::FileStorage fs(trainPath, cv::FileStorage::READ);
	cv::Mat temp;
	for(int i = 0; i < amountOfClasses; i++){
		temp.release();
		string dataname = "trainingdata" + modestring+ boost::lexical_cast<string>(i);
		fs[dataname] >> temp;
		trainingdata.push_back(temp);
	}
	fs.release();
}

int RFClassifier::addTrainingPoint( cv::Mat featureVector, int classNum )
{
	cout << "Adding training point" << endl; 
	if (classNum == -1) {
		trainingdata.push_back(featureVector);
		classNum = trainingdata.size();
		amountOfClasses++;
	} else {
		for(int i = 0; i < 30; i++ )
			cv::hconcat(trainingdata.at(classNum), featureVector, trainingdata.at(classNum));
	}

	return classNum;
}



int RFClassifier::predict(cv::Mat input){
	return static_cast<int>(treestructure->predict(input,cv::Mat()));
}

void RFClassifier::trainTree()
{
	trainTree(trainingdata);
}

void RFClassifier::trainTree(vector<cv::Mat> trainingData){
	
	cout << "Starting RF training.."<< endl;
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
	cout << "Treestructure trained" << endl;
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