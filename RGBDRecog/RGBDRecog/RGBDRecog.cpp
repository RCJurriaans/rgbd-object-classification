// RGBDRecog.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <crtdbg.h>

//Include files

//used for compilation
#include "stdafx.h" 

//include the openCV libraries -> moved to stdafx
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>

//Include other libraries
//#include "ImageWrapper.h" // used for directly accessing ipl images
#include "FeatureVector.h" //feature vector contains SIFT features
#include <string>
//#include <boost\lexical_cast.hpp> //to quickly lexical cast integers //moved to stdafx

#include "Winbase.h"

using namespace std;
using namespace cv; //opencv namespace




//settings
enum classes {AIRPLANES, CARS, FACES, MOTORBIKES}; //names of the classes
enum colorsettings {NORMAL, HUE, OPPONENT}; //name of the color modes
string classNames[4] = {"airplanes", "cars", "faces", "motorbikes"}; //strings of names of the classes
const int classMax[4] = {500,465,400,500};
const int numTestIm[4] = {50,50,50,50};
const int amountOfClasses = 4; 
const int cookBookNumImg[4] = {150,115,50,150}; //number of images used to create the dictionary. The first 'this amount' images are used from the folders
const int SIFTThreshScale = 4; //Threshold for the amount of points that sift finds for each image, 4 gives around 450 points...
const int dicsize = 500; //size of the codebook to use for SIFT features
//#define _CRTDBG_MAP_ALLOC //used for memory leak checking in debug mode
#define IMGEXTENSION ".jpg" //extension that is used of the images


//converts a number to a fixed length string, adding zeros in front
string convertNumberToFLString(int length, int number){

	int tempnum = number;
	length--;
	string returnvalue;
			while(tempnum/10 != 0){
				tempnum = tempnum/10;
				length--;
			}
			for(int k = 0; k < length; k++){
				returnvalue += "0";
			}
			returnvalue += boost::lexical_cast<string>(number);
	return returnvalue;
}

//generate the codebook for SIFT features, mode specifies which color scheme is used
void createCookbook(const int mode){
	string imagePath; //path where the images are
	string filePath; //filepath for each image
	Mat tempinput; //temp
	vector<Mat> input; //stores the image data
	vector<Mat> planes; //temp file for splitting the image data
	FeatureVector* fv = new FeatureVector; //this contains all features taht are extacted
	SIFT detector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE ); //sift class, ugly initialization of all parameters...
    vector<KeyPoint> keypoints; //temp variable that contains the keypoints found for each img
	Mat descriptors; //temp variable used to store the extracted SIFT descriptors for every image
	SiftDescriptorExtractor* SDE = new cv::SiftDescriptorExtractor(); //Sift descriptor extraction class
	OpponentColorDescriptorExtractor des(SDE); //written by van de Sande... extracts the opponent SIFT data
	Mat grayimg; //temp used to store the grayimg in for Opponent color 

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "class: " << i << endl;
		for(int j = 1; j <= cookBookNumImg[i]; j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + IMGEXTENSION; //get the proper filename
			cout << "image: " << j << endl;
			if(mode == NORMAL){ //every step cleans up all the matrices, and adds new image data
				input.clear();
				input.push_back(cv::imread(imagePath,0)); //Load as grayscale image
			}else if (mode == HUE){
				input.clear();
				tempinput.release();
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input.push_back(planes[0]);
				input.push_back(planes[2]);
			}else if (mode == OPPONENT){
				tempinput.release();
				tempinput = cv::imread(imagePath);
			}
			if (mode == NORMAL){ //use the gotten image data to compute the features, and add them to the feature vector
				descriptors.release(); //still clear all data before reinitializing
				detector(input[0],Mat(),vector<KeyPoint>(),descriptors);
				fv->AddFeatures(descriptors);
			} else if(mode == HUE){
				descriptors.release();
				keypoints.clear();
				detector(input[1],Mat(),keypoints,Mat());
				detector(input[0],Mat(),keypoints,descriptors,true);
				fv->AddFeatures(descriptors);
			} else if(mode == OPPONENT){
				descriptors.release();
				keypoints.clear();
				grayimg.release();
				cvtColor(tempinput,grayimg,CV_RGB2GRAY);
				detector(grayimg,Mat(),keypoints,Mat());
				des.compute(tempinput,keypoints,descriptors);
				fv->AddFeatures(descriptors);
			}
		}
	}
	cout << "Running kmeans" << endl;
	Mat* codebook = fv->kmeans(dicsize); //run kmeans on the data for dicsize clusters
	FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::WRITE); //store the codebook to a yaml file
	fs << "codebook" + boost::lexical_cast<string>(mode) << *codebook;
	
	//delete all data... just for security :/
	fv->~FeatureVector();
	tempinput.release();
	for(unsigned int i = 0; i < input.size(); i++)
	{
		input[i].release();
	}
	for(unsigned int i = 0; i < planes.size(); i++)
	{
		planes[i].release();
	}
	//detector....
	//for(int i = 0; i < keypoints.size(); i++)
	//{
	//	keypoints[i].release();
	//}
	descriptors.release();
	des.empty();
	grayimg.release();
	delete fv;
	delete codebook;
	_CrtDumpMemoryLeaks();
}

vector<Mat> generateTrainingData(FeatureVector codebook, int mode){
	string imagePath; //path where the images are
	string filePath; //filepath for each image
	Mat tempinput; //temp
	vector<Mat> input; //stores the image data
	vector<Mat> planes; //temp file for splitting the image data
	SIFT detector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE ); //sift class, ugly initialization of all parameters...
    vector<KeyPoint> keypoints; //temp variable that contains the keypoints found for each img
	Mat descriptors; //temp variable used to store the extracted SIFT descriptors for every image
	//SiftDescriptorExtractor SDE; //Sift descriptor extraction class
	//OpponentColorDescriptorExtractor des(&SDE); //written by van de Sande... extracts the opponent SIFT data
	Mat grayimg; //temp used to store the grayimg in for Opponent color 
	vector<Mat> histogram;

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "class: " << i << endl;
		for(int j = cookBookNumImg[i]+1; j <= classMax[i]; j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + IMGEXTENSION; //get the proper filename
			cout << "image: " << j << endl;
			if(mode == NORMAL){ //every step cleans up all the matrices, and adds new image data
				input.clear();
				input.push_back(cv::imread(imagePath,0)); //Load as grayscale image
			}else if (mode == HUE){
				input.clear();
				tempinput.release();
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input.push_back(planes[0]);
				input.push_back(planes[2]);
			}else if (mode == OPPONENT){
				tempinput.release();
				tempinput = cv::imread(imagePath);
			}
			if (mode == NORMAL){ //use the gotten image data to compute the features, and add them to the feature vector
				descriptors.release(); //still clear all data before reinitializing
				detector(input[0],Mat(),vector<KeyPoint>(),descriptors);
			} else if(mode == HUE){
				descriptors.release();
				keypoints.clear();
				detector(input[1],Mat(),keypoints,Mat());
				detector(input[0],Mat(),keypoints,descriptors,true);
			} else if(mode == OPPONENT){
				descriptors.release();
				keypoints.clear();
				grayimg.release();
				cvtColor(tempinput,grayimg,CV_RGB2GRAY);
				detector(grayimg,Mat(),keypoints,Mat());
				//des.compute(tempinput,keypoints,descriptors);
			}
			//descriptor now contains the found feature descriptors,
			//we throw these to the codebook to generate a histogram
			//histogram.release();
			if(j == cookBookNumImg[i]+1){
				histogram.push_back(codebook.GenHistogram(descriptors));
			}
			else{
				hconcat(histogram[i],codebook.GenHistogram(descriptors),histogram[i]);
			}
		}
	}
	return histogram;
}

void predictTestSet(CvRTrees * treestructure, int mode, FeatureVector codebook){
	string imagePath; //path where the images are
	string filePath; //filepath for each image
	Mat tempinput; //temp
	vector<Mat> input; //stores the image data
	vector<Mat> planes; //temp file for splitting the image data
	FeatureVector* fv = new FeatureVector; //this contains all features taht are extacted
	SIFT detector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE ); //sift class, ugly initialization of all parameters...
    vector<KeyPoint> keypoints; //temp variable that contains the keypoints found for each img
	Mat descriptors; //temp variable used to store the extracted SIFT descriptors for every image
	//SiftDescriptorExtractor SDE; //Sift descriptor extraction class
	//OpponentColorDescriptorExtractor des(&SDE); //written by van de Sande... extracts the opponent SIFT data
	Mat grayimg; //temp used to store the grayimg in for Opponent color 
	Mat results;

	float** predictions = new float*[amountOfClasses];
	for(int i = 0; i < amountOfClasses; i++){
		predictions[i] = new float [numTestIm[i]];
	}

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_test\\"; //go to the classname folder
		cout << "class: " << i << endl;
		for(int j = 1; j < numTestIm[i]; j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + IMGEXTENSION; //get the proper filename
			cout << "image: " << j << endl;
			if(mode == NORMAL){ //every step cleans up all the matrices, and adds new image data
				input.clear();
				input.push_back(cv::imread(imagePath,0)); //Load as grayscale image
			}else if (mode == HUE){
				input.clear();
				tempinput.release();
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input.push_back(planes[0]);
				input.push_back(planes[2]);
			}else if (mode == OPPONENT){
				tempinput.release();
				tempinput = cv::imread(imagePath);
			}
			if (mode == NORMAL){ //use the gotten image data to compute the features
				descriptors.release(); //still clear all data before reinitializing
				detector(input[0],Mat(),vector<KeyPoint>(),descriptors);
				results.release();
				results = codebook.GenHistogram(descriptors);
				predictions[i][j] = treestructure->predict(results,Mat());
			} else if(mode == HUE){
				descriptors.release();
				keypoints.clear();
				detector(input[1],Mat(),keypoints,Mat());
				detector(input[0],Mat(),keypoints,descriptors,true);
			} else if(mode == OPPONENT){
				descriptors.release();
				keypoints.clear();
				grayimg.release();
				cvtColor(tempinput,grayimg,CV_RGB2GRAY);
				detector(grayimg,Mat(),keypoints,Mat());
				//des.compute(tempinput,keypoints,descriptors);
				fv->AddFeatures(descriptors);
			}
		}
	}
	vector<float> accuracy;
	for(int i = 0; i < amountOfClasses; i++){
		float sum = 0;
		for(int j = 0; j < numTestIm[i]; j++){
			if(predictions[i][j] == static_cast<float>(i)){
				sum++;
			}
		}
		sum = sum/numTestIm[i];
		accuracy.push_back(sum);
	}


	for(int i = 0; i < amountOfClasses; i++){
		delete [] predictions[i];
	}
	delete [] predictions;
 }

int _tmain(int argc, _TCHAR* argv[])
{
	int mode = NORMAL; //specify the color mode used, either NORMAL, HUE or OPPONENT

	//createCookbook(mode);
	
	//load the codebook data
	FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::READ);
	Mat M; fs["codebook" + boost::lexical_cast<string>(mode)] >> M;
	fs.release();
	FeatureVector codebook;
	codebook.AddFeatures(M);
	codebook.TrainkNN();//train kNN method for evaluating close descriptors
	

	//Mat img = imread("nemo1.jpg");
	//imshow( "name", img);
	//cvWaitKey(0);




	vector<Mat> trainingdata = generateTrainingData(codebook, mode);
	vector<int> classNumbers;
	for(unsigned int i = 0; i < trainingdata.size(); i++){
		classNumbers.push_back(trainingdata[i].rows);
	}
	Mat mergedData;
	mergedData = trainingdata[0];
	for(int i = 1; i < amountOfClasses; i++){
		hconcat(mergedData,trainingdata[i],mergedData);
	}

	FileStorage fs2("trainingdata" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::WRITE); //store the codebook to a yaml file
	for(int i = 0; i < amountOfClasses;i++){
		fs2 << "trainingdata" + boost::lexical_cast<string>(mode) + boost::lexical_cast<string>(i)  << trainingdata[i];
	}
	fs2.release();






	
	vector<Mat> trainingdata2;
	FileStorage fs3("trainingdata" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::READ);
	Mat temp;
	for(int i = 0; i < amountOfClasses; i++){
		temp.release();
		fs3["trainingdata" + boost::lexical_cast<string>(mode)+ boost::lexical_cast<string>(i)] >> temp;
		trainingdata2.push_back(temp);
	}
	Mat mergedData2;
	mergedData2 = trainingdata2[0];
	for(int i = 1; i < amountOfClasses; i++){
		hconcat(mergedData2,trainingdata2[i],mergedData2);
	}
	fs2.release();
	cout << "training tree" << endl;
	CvRTrees* treestructure = new CvRTrees();
	Mat responses(mergedData2.cols,1,mergedData2.type());
	for(int i = 0; i < amountOfClasses;i++){
		int multifactor=0;
		for(int j = 0; j < i; j++){
			multifactor+= classMax[j]-cookBookNumImg[i];
		}
		for(int j = 0; j < trainingdata2[i].cols; j++){
			responses.at<float>(j+multifactor) = static_cast<float>(i);
		}
	}

	treestructure->train(mergedData2,CV_COL_SAMPLE,responses,Mat(),Mat(),Mat(),Mat(),CvRTParams());
	//CvFileStorage fs3("randomforestdata" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::WRITE);
	string filename = "randomforestdata" + boost::lexical_cast<string>(mode);
	string dataname = "randomforestdata" + boost::lexical_cast<string>(mode) + ".yml";
	treestructure->write(cvOpenFileStorage(dataname.c_str(), 0, CV_STORAGE_WRITE_TEXT ), filename.c_str());


	cout << "loading tree structure" << endl;
	string filename2 = "randomforestdata" + boost::lexical_cast<string>(mode);
	string dataname2 = "randomforestdata" + boost::lexical_cast<string>(mode) + ".yml";
	CvRTrees* treestructure2 = new CvRTrees();
	treestructure2->read(cvOpenFileStorage(dataname2.c_str(), 0, CV_STORAGE_READ ), cvGetFileNodeByName(cvOpenFileStorage(dataname2.c_str(), 0, CV_STORAGE_READ ),0,filename2.c_str()));


	//_CrtDumpMemoryLeaks(); //used to check memory leakages in debug mode, will output to the output screen in msvc2010
	cout << "predicting test data" << endl;
	predictTestSet(treestructure2, mode, codebook);

		cout << "done" << endl;

	return 0;
}
