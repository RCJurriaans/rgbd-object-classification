#pragma once
#include <string>
#include "FeatureVector.h"
#include "FeatureExtractor.h" 
#include "RFClassifier.h"
using namespace std;
//using namespace cv;



class RFClass
{
public:
	RFClass(void);
	~RFClass(void);

	void menu();
private:
	void createCodebookMenu();
	void rfTrainigmenu();
	void createCodebook(int mode);
	string convertNumberToFLString(int length, int number);
	string ifBoolReturnChar(bool in, string out);
	void addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector, FeatureVector * codebooks,cv::Mat & descriptors, int mode);
	void trainModelMenu();
	void trainModel(vector<int> mode);
	void generateRandomForest(vector<int> mode);

	void rfTesting(vector<int> mode);

	void rfTestingmenu();

	int amountOfClasses;
	string fileExtension;
	vector<string> classNames;
	vector<int> trainigPicNum;
	vector<int> testPicNum;
	int SIFTThreshScale;
	int dicsize;

	static vector<string> colornames;

	FeatureExtractor* featureExtractor;
	RFClassifier * rfclassifier;
};
