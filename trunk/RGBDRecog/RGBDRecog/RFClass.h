#pragma once
#include <string>
#include "FeatureVector.h"
using namespace std;
using namespace cv;



class RFClass
{
public:
	RFClass(void);
	~RFClass(void);

	void menu();
private:
	void createCodebookMenu();
	void createCodebook(int mode);
	string convertNumberToFLString(int length, int number);
	string ifBoolReturnChar(bool in, string out);
	void addDescriptor(bool & firstadded, Mat & tempfeaturevector, FeatureVector * codebooks,Mat & descriptors, int mode);
	void trainModelMenu();
	void trainModel(vector<int> mode);
	int amountOfClasses;
	string fileExtension;
	vector<string> classNames;
	vector<int> traingPicNum;
	vector<int> testPicNum;
	int SIFTThreshScale;
	int dicsize;
	int amountOfPossibleFeatures;

	static vector<string> colornames;
};
