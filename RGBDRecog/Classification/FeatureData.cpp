#include "StdAfx.h"
#include "FeatureData.h"


FeatureData::FeatureData(void)
{
	// edit the names and possible feature parameters for the whole function here!
	amountOfFeatures = 7;
	featureNames = new string[amountOfFeatures];
	featureNames[0] = "SIFT"; 
	featureNames[1] = "Hue SIFT";
	featureNames[2] = "Opponent SIFT";
	featureNames[3] = "Normal SURF";
	featureNames[4] = "Hue SURF";
	featureNames[5] = "Opponent SURF";
	featureNames[6] = "Color Histogram";
}


FeatureData::~FeatureData(void)
{
	delete [] featureNames;
}
