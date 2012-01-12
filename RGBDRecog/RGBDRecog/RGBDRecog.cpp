// RGBDRecog.cpp : Defines the entry point for the console application.
//


//Include files

//used for compilation
#include "stdafx.h" 

//include the openCV libraries
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//Include other libraries
#include "ImageWrapper.h" // used for directly accessing ipl images
#include "FeatureVector.h" //feature vector contains SIFT features
#include <string>
#include <boost\lexical_cast.hpp> //to quickly lexical cast integers

using namespace std;
using namespace cv; //opencv namespace

enum classes {AIRPLANES, CARS, FACES, MOTORBIKES}; //names of the classes
string classNames[4] = {"airplanes", "cars", "faces", "motorbikes"}; //strings of names of the classes
const int amountOfClasses = 4;
const int cookBookNumImg = 100; //number of images used to create the dictionary. The first 'this amount' images are used from the folders
#define IMGEXTENTION ".jpg";

void createCookbook(){
	string imagePath;
	string filePath;
	int integer;
	int zeros = 0;
	for(int i = 0; i < amountOfClasses; i++){
		filePath = classNames[i];
		filePath += "_train\\";
		for(int j = 1; j <= cookBookNumImg; j++){
			imagePath = filePath;
			imagePath += "img";
			integer = j;
			zeros = 2;
			while(integer/10 != 0){
				integer = integer/10;
				zeros--;
			}
			for(int k = 0; k < zeros; k++){
				imagePath += "0";
			}
			imagePath += boost::lexical_cast<string>(j);
			imagePath += IMGEXTENTION;

			const cv::Mat input = cv::imread(imagePath); //Load as grayscale
			imshow("testing",input);
			cvWaitKey(0);
		}
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
    const cv::Mat input = cv::imread("nemo1.jpg", 0); //Load as grayscale

    SIFT detector;
    vector<KeyPoint> keypoints;
	Mat descriptors = Mat();
    detector(input,Mat(),keypoints,descriptors);
    // Add results to image and save.
    Mat output;
    drawKeypoints(input, keypoints, output);
    imwrite("sift_result.jpg", output);
	imshow("testing",output);
	cvWaitKey(0);

	FeatureVector fv;
	fv.AddFeatures(descriptors);
	fv.AddFeatures(descriptors);

	createCookbook();


	return 0;
}
