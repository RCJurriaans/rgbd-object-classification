#include "StdAfx.h"
#include "FeatureVector.h"

//constructor
FeatureVector::FeatureVector(void)
{
	numFeatures = 0;
	features = new Mat;
}

//destructor
FeatureVector::~FeatureVector(void)
{
	features->release();
}

//add descriptorIn features to this class' feature list
void FeatureVector::AddFeatures(Mat descriptorIn){
	if(features->rows == 0){
		*features = descriptorIn; //no features as of yet
	}
	else{
		vconcat(descriptorIn,*features,*features); //there are features, concat them
	}
}

//apply kmeans with dicsize clusters on the feature set
Mat* FeatureVector::kmeans(int dicsize){
	Mat* centers = new Mat;
	//features is the list with features, the second argument is for returning the labels of each points (not used)
	//Termcriteria has in the 2nd argument the amount of steps (ignore the third, the fifth variable has the amount of random
	//reinitializations (not needed because of the used initialization method), and KMEANS_PP_CENTERS implements a smart
	//initialization algorithm. In the end, centers has the cluster centers, of which a dicsize amount is found
	cv::kmeans(*features,dicsize,Mat(),TermCriteria(1,10,1),1,KMEANS_PP_CENTERS,*centers);
	
	return centers;
}