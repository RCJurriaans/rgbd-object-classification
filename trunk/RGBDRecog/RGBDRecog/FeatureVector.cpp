#include "StdAfx.h"
#include "FeatureVector.h"


FeatureVector::FeatureVector(void)
{
	numFeatures = 0;
	features = Mat();
}


FeatureVector::~FeatureVector(void)
{
}

void FeatureVector::AddFeatures(Mat descriptorIn){
	if(features.rows == 0){
		features = descriptorIn;
	}
	else{
		vconcat(descriptorIn,features,features);
	}
}