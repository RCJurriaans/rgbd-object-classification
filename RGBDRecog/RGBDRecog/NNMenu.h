
#include "Settings.h"
#include "FeatureExtractor.h"
#include <NBNN.h>

class NNMenu
{
public:
	NNMenu(Settings * set);
	~NNMenu(void);

	void trainData();
	void menu();
	void NNTesting();
	
	cv::Rect getDatasetROINN(string folderPath, int j);
	cv::Rect readRectNN(const string filePath);
	vector<int> DrawWithReplacementNN(int maxrange, int amount);

	int amountOfClasses;
	string fileExtension;
	vector<string> classNames;
	vector<int> trainigPicNum;
	vector<int> testPicNum;

	vector<vector<int>> trainingdataID;
	vector<vector<int>> testdataID;

	FeatureExtractor* featureExtractor;
	Settings * settings;
	NBNN * nbnn;
	cv::RNG * rng;
};