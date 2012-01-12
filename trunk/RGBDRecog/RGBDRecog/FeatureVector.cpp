#include "StdAfx.h"
#include "FeatureVector.h"


FeatureVector::FeatureVector(void)
{
	numFeatures = 0;
	features = new Mat;
}


FeatureVector::~FeatureVector(void)
{
	features->release();
}

void FeatureVector::AddFeatures(Mat descriptorIn){
	if(features->rows == 0){
		*features = descriptorIn;
	}
	else{
		vconcat(descriptorIn,*features,*features);
	}
}

Mat* FeatureVector::kmeans(int dicsize){
	Mat* centers = new Mat;
	cv::kmeans(*features,dicsize,Mat(),TermCriteria(1,10,1),1,KMEANS_PP_CENTERS,*centers);
	
	return centers;
}