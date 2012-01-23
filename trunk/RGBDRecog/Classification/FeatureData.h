#pragma once

using namespace std;
class FeatureData
{
public:
	FeatureData(void);
	~FeatureData(void);

	string* featureNames; //contains the names of all features
	int amountOfFeatures; //the amount of total possible features that can be used
};

