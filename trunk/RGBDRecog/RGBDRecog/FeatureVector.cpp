#include "StdAfx.h"
#include "FeatureVector.h"
#include <ml.h>
#include <highgui.h>

//constructor
FeatureVector::FeatureVector(void)
{
	features = new Mat;
	knn = new CvKNearest();
}

//destructor
FeatureVector::~FeatureVector(void)
{
	features->release();
	//delete knn;
	knn->clear();
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

void FeatureVector::TrainkNN(){
	Mat trainClasses(features->rows,1,features->type());
	for(int i =0; i < features->rows; i++){
		trainClasses.at<float>(i,0) = i;
	}
	
	CvMat tempfeats = *features;
	CvMat * feats = &tempfeats;
	CvMat temptrain = trainClasses;
	CvMat * train = &temptrain;
	knn->train( feats, train, 0, false, 1 );
}

 // take a set of feature descriptors, and create a codebook histogram of these
Mat FeatureVector::GenHistogram(const Mat descriptors){
	CvMat* nearests = cvCreateMat( descriptors.rows, 1, CV_32FC1);
	CvMat tempdesc = descriptors;
	CvMat * desc = &tempdesc;
	knn->find_nearest(desc,1,0,0,nearests,0);

	Mat nearest = nearests;
	MatND histogram;
	float hranges[] = { 0, 500 };
    const float* ranges[] = { hranges };
	int channels[] = {0};
	int hbins = 500;
	int histSize[] = {hbins};
	calcHist(&nearest,1,channels,Mat(),histogram,1,histSize,ranges,true,false);
	
	normalize(histogram,histogram);

	//nearest.release();
	//delete ranges;
	return histogram;
}