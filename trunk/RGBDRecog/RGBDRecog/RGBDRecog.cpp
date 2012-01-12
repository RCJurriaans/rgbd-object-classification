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

#include "Winbase.h"

using namespace std;
using namespace cv; //opencv namespace

enum classes {AIRPLANES, CARS, FACES, MOTORBIKES}; //names of the classes
enum colorsettings {NORMAL, HUE, OPPONENT};
string classNames[4] = {"airplanes", "cars", "faces", "motorbikes"}; //strings of names of the classes
const int amountOfClasses = 4;
const int cookBookNumImg = 100; //number of images used to create the dictionary. The first 'this amount' images are used from the folders
#define IMGEXTENSION ".jpg";

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
	int integer;
	int zeros = 0;
	cv::Mat tempinput;
	cv::Mat input;
	vector<Mat> planes;

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR");
		filePath += "\\" + classNames[i] + "_train\\";
		//filePath += "_train\\";
		for(int j = 1; j <= cookBookNumImg; j++){
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + IMGEXTENSION;

			if(mode == NORMAL){
				input = cv::imread(imagePath,0); //Load as grayscale image
			}else if (mode == HUE){
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input = planes[1];
			}else if (mode == OPPONENT){
				cv::Mat tempinput = cv::imread(imagePath);
			}
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
	//imshow("testing",output);
	cvWaitKey(0);

	FeatureVector fv;
	fv.AddFeatures(descriptors);
	fv.AddFeatures(descriptors);

	createCookbook(HUE);


	return 0;
}
