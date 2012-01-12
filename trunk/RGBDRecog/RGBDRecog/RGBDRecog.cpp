// RGBDRecog.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <crtdbg.h>

//Include files

//used for compilation
#include "stdafx.h" 

//include the openCV libraries
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>

//Include other libraries
//#include "ImageWrapper.h" // used for directly accessing ipl images
#include "FeatureVector.h" //feature vector contains SIFT features
#include <string>
//#include <boost\lexical_cast.hpp> //to quickly lexical cast integers

#include "Winbase.h"

using namespace std;
using namespace cv; //opencv namespace

enum classes {AIRPLANES, CARS, FACES, MOTORBIKES}; //names of the classes
enum colorsettings {NORMAL, HUE, OPPONENT};
string classNames[4] = {"airplanes", "cars", "faces", "motorbikes"}; //strings of names of the classes
const int amountOfClasses = 4;
const int cookBookNumImg = 250; //number of images used to create the dictionary. The first 'this amount' images are used from the folders
#define IMGEXTENSION ".jpg";
const int SIFTThreshScale = 4;
const int dicsize = 500;
#define _CRTDBG_MAP_ALLOC
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

void createCookbook(const int mode){
	string imagePath;
	string filePath;
	Mat tempinput;
	vector<Mat> input;
	vector<Mat> planes;
	FeatureVector* fv = new FeatureVector;
	SIFT detector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE );
    vector<KeyPoint> keypoints;
	Mat descriptors;
	SiftDescriptorExtractor SDE;
	OpponentColorDescriptorExtractor des(&SDE);
	Mat grayimg;

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR");
		filePath += "\\" + classNames[i] + "_train\\";
		cout << "class: " << i << endl;
		for(int j = 1; j <= cookBookNumImg; j++){
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + IMGEXTENSION;
			cout << "image: " << j << endl;
			if(mode == NORMAL){
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
				//convertBGRImageToOpponentColorSpace(tempinput,input);
			}
			if (mode == NORMAL){
				descriptors.release();
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
	Mat* codebook = fv->kmeans(dicsize);
	FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::WRITE);
	fs << "codebook" + boost::lexical_cast<string>(mode) << *codebook;
	

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

int _tmain(int argc, _TCHAR* argv[])
{
	int mode = NORMAL;

	createCookbook(mode);
	FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::READ);
	Mat M; fs["codebook" + boost::lexical_cast<string>(mode)] >> M;

	_CrtDumpMemoryLeaks();
	return 0;
}
