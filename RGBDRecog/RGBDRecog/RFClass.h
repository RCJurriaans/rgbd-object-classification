#pragma once
#include <string>
#include "FeatureVector.h"
#include "FeatureExtractor.h" 
#include "RFClassifier.h"
#include "Settings.h"
using namespace std;
//using namespace cv;



class RFClass
{
public:
	RFClass(Settings * set);
	~RFClass(void);

	void menu();

	static string convertNumberToFLString(int length, int number);
private:
	void createCodebookMenu();
	void rfTrainigmenu();
	void createCodebook(int mode);
	
	string ifBoolReturnChar(bool in, string out);
	void addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector, FeatureVector * codebooks,cv::Mat & descriptors, int mode);
	void trainModelMenu();
	void trainModel();
	void generateRandomForest();

	void rfTesting();

	void rfTestingmenu();

	vector<int> DrawWithReplacement(int maxrange, int amount);

	cv::Rect readRect(const string filePath);
	cv::Rect getDatasetROI(string folderpath, int j);

	int amountOfClasses;
	string fileExtension;
	vector<string> classNames;
	vector<int> trainigPicNum;
	vector<int> testPicNum;

	vector<vector<int>> trainingdataID;
	vector<vector<int>> testdataID;

	int SIFTThreshScale;
	int dicsize;

	static vector<string> colornames;

	FeatureExtractor* featureExtractor;
	RFClassifier * rfclassifier;
	Settings * settings;
	cv::RNG * rng;
};
