#pragma once

#include "SegmentCloud.h"

using namespace std;

class DataSegmenter
{
public:
	DataSegmenter(void);
	~DataSegmenter(void);

	//loads the menu in which all data segmentation options can be selected
	void menu();
private:
	//generates bounding boxes cv::Rect info for each of the images in the training set
	void generateBoundingBoxes();
	SegmentCloud * segmentation;

	//dataset info, remove this
	int amountOfClasses;
	string fileExtension;
	vector<string> classNames;
	vector<int> trainigPicNum;
	vector<int> testPicNum;

	cv::Rect readRect(const string filePath);
	void writeRect(const string filePath,const cv::Rect rect);

	string convertNumberToFLString(int length, int number);


};

