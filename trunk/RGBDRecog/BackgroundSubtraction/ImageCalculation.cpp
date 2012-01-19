#include "StdAfx.h"
#include "ImageCalculation.h"
#include "ConstantDefinitions.h"
#include "ImageAccess.h"
#include "DisjointSet.h"

#include <cv.h>	
#include <cvaux.h>
#include <highgui.h>
using namespace cv;

#define _X_COORD 0
#define _Y_COORD 1
#define _CLUSTER_NUMBER 2

ImageCalculation::ImageCalculation(void)
{
	mask = NULL;
	ClusterMap = NULL;
	ClusterMapSize = cvSize(0,0);
	clusterNumber = 0;
	number_of_white_pixels = 0;
	whitePixelInformationSize = 0;
	whitePixelInformation = NULL;
	clusterSizes = NULL;
	clusterSizesSize = 0;
	largestClusterNumbers = NULL;
	largestClusterNumbersSizes = NULL;
	largestClusterNumbersSize = 0;
	means = NULL;
	meansSize = 0;
	regiosize = 0;
	regions = NULL;
}


ImageCalculation::~ImageCalculation(void)
{
	if(mask!= NULL){
		cvReleaseImage(&mask );
	}
	DestroyClusterMap();
	DestroyWPI();
	DetroyClusterSizes();
	DestroylargestClusterNumbers();
	DestroyMeans();
	DestroyRegions();
}

void ImageCalculation::DestroyClusterMap(){
	if(ClusterMap != NULL){
		for(int i = 0; i < ClusterMapSize.height; i++){
			if(ClusterMap[i] != NULL){
				delete [] ClusterMap[i];
				ClusterMap[i] = NULL;
			}
		}
		delete ClusterMap;
		ClusterMap = NULL;
	}
}

void ImageCalculation::InitializeClusterMap(int height, int width){
	DestroyClusterMap();

	ClusterMap = new unsigned int *[height];
	for(int i = 0; i < height; i++){
		ClusterMap[i] = new unsigned int[width];
	}
	ClusterMapSize = cvSize(width, height);
}


void ImageCalculation::InitializeWPI(int whitePixelAmount){
	DestroyWPI();
	whitePixelInformation = new unsigned int * [3];
	for(int i = 0; i < 3; i++){
		whitePixelInformation[i] = new unsigned int [whitePixelAmount];
	}
	whitePixelInformationSize = whitePixelAmount;
}

void ImageCalculation::DestroyWPI(){ //deletes the whole array
	if(whitePixelInformation != NULL){
		for(int i = 0; i < 3; i++){
			if(whitePixelInformation[i] != NULL){
				delete[] whitePixelInformation[i];
				whitePixelInformation[i] = NULL;
			}
		}
		delete [] whitePixelInformation;
		whitePixelInformation = NULL;
	}
}

void ImageCalculation::InitializeClusterSizes(int numberofclusters){
	DetroyClusterSizes();
	clusterSizes = new unsigned int [numberofclusters];
	clusterSizesSize = numberofclusters;
}

void ImageCalculation::DetroyClusterSizes(){
	if(clusterSizes != NULL){
		delete [] clusterSizes;
		clusterSizes = NULL;
	}
}

void ImageCalculation::InitializelargestClusterNumbers(int N){
	DestroylargestClusterNumbers();
	largestClusterNumbers = new unsigned int [N];
	largestClusterNumbersSizes = new unsigned int [N];
	largestClusterNumbersSize = N;
}

void ImageCalculation::DestroylargestClusterNumbers(){
	if(largestClusterNumbers != NULL){
		delete [] largestClusterNumbers;
		largestClusterNumbers = NULL;
	}
	if(largestClusterNumbersSizes != NULL){
		delete [] largestClusterNumbersSizes;
		largestClusterNumbersSizes = NULL;
	}
}

void ImageCalculation::InitializeMeans(const int N){
	DestroyMeans();
	means = new unsigned int * [2];
	means[_X_COORD] = new unsigned int [N];
	means[_Y_COORD] = new unsigned int [N];
	meansSize = N;
}

void ImageCalculation::DestroyMeans(){
	if(means != NULL){
		if(means[_X_COORD] != NULL){
			delete [] means[_X_COORD];
			means[_X_COORD] = NULL;
		}
		if(means[_Y_COORD] != NULL){
			delete [] means[_Y_COORD];
			means[_Y_COORD] = NULL;
		}
		delete [] means;
		means = NULL;
	}
}

void ImageCalculation::Calculate(IplImage* thisImage, int numberOfMarkers){ //if the size do not match, change the frames
	//create a new mask image, but only if the sizes of the current are not compatible
	if(mask != NULL){
		if((mask->width != thisImage->roi->width || mask->height != thisImage->roi->height)){
			cvReleaseImage(&mask); //delete and make mask the size of the roi of the image
			mask = cvCreateImage(cvSize(thisImage->roi->width,thisImage->roi->height),thisImage->depth,thisImage->nChannels);
		}
	}else{//make mask the size of the roi of the image
		mask = cvCreateImage(cvSize(thisImage->roi->width,thisImage->roi->height),thisImage->depth,thisImage->nChannels);
	}

	
	//copy thisImage to mask, only copies the ROI!
	cvCopy(thisImage, mask, NULL);
	//smooth it out, removing single pixels
	cvSmooth(mask, mask, CV_MEDIAN, _SMOOTHRANGE);


	//cvShowImage("mask",mask);

	//calculate the means of the largest clusters
	DetermineClusterNumbers();
	DetermineWhitePixelCoords();
	DetermineClusterSizes();
	FindNBiggestClusters(numberOfMarkers);
	DetermineNMeans(numberOfMarkers);
}


//function determines 4 connectivity connected components of the mask image.
//Stored in ClusterMap is a label number for each connected component. same number means pixels are connected
void ImageCalculation::DetermineClusterNumbers(){
	DisjointSet disjointSet; //used to store the connected component labels in for the backward pass
							 //which labels are equal to which labels etc.

	//If the current ClusterMap is smaller than the mask, we need to make it bigger
	if(ClusterMapSize.height < mask->height || ClusterMapSize.width < mask->width){
		InitializeClusterMap(mask->height, mask->width);
	}

	//zero the ClusterMapArray part we'll be using
	for(int i = 0; i < mask->height; i++){
		for(int j = 0; j < mask->width; j++){
			ClusterMap[i][j] = 0; 
		}
	}

	BwImage enter(mask); //so we can directly quick access mask

	int top_number_label; //these 4 used to store
	int left_number_label; //the labels relative to the pixel we're currently looking at
	int top_number_pixel;
	int left_number_pixel;
	int cluster_counter = 1; //number that we assign to each cluster
	int white_pixel_counter = 0;  //used to calculate the amount of white pixels in the image
	
	//FIRST PASS
	//first pixel
	if(enter[0][0] != 0){
		ClusterMap[0][0] = cluster_counter;
		cluster_counter++; //next new assignment of clusters is one higher
		disjointSet.AddSet(); //add one set to DisjointSet (number 1)
		white_pixel_counter++; //one extra white pixel
	}
	//first row
	for(int j = 1; j < mask->width; j++){
		if(enter[0][j] != 0){
			white_pixel_counter++;
			if(ClusterMap[0][j-1] != 0){ //part of cluster to the left
				ClusterMap[0][j] = ClusterMap[0][j-1]; //dont look at the top
			}
			else{ //new cluster
				ClusterMap[0][j] = cluster_counter;
				cluster_counter++;
				disjointSet.AddSet();
			}
		}
	}
	//first column
	for(int i = 1; i < mask->height; i++){
		if(enter[i][0] != 0){
			white_pixel_counter++;
			if(ClusterMap[i-1][0] != 0){ //part of cluster to the top
				ClusterMap[i][0] = ClusterMap[i-1][0];
			}
			else{ //new cluster
				ClusterMap[i][0] = cluster_counter;
				cluster_counter++;
				disjointSet.AddSet();
			}
		}
	}
		//rest area
	for(int i = 1; i < mask->height; i++){
		for(int j = 1; j < mask->width; j++){
			if(enter[i][j] != 0){
				white_pixel_counter++;
				//Get the neighboring elements of the current element
				top_number_label = ClusterMap[i-1][j];
				left_number_label = ClusterMap[i][j-1];
				top_number_pixel = enter[i-1][j];
				left_number_pixel = enter[i][j-1];
				
				if(top_number_pixel == 0 && left_number_pixel == 0)
				{//If there are no neighbors, uniquely label the current element and continue
					ClusterMap[i][j] = cluster_counter;
					cluster_counter++;
					disjointSet.AddSet();
				}else{
				//Otherwise, find the neighbor with the smallest label and assign it to the current element
					if(top_number_label <1){
						ClusterMap[i][j] = left_number_label;
					}else if(left_number_label < 1){
						ClusterMap[i][j] = top_number_label;
					}else if(top_number_label < left_number_label){
						ClusterMap[i][j] = top_number_label;
					}else{
						ClusterMap[i][j] = left_number_label;
					}
				}
				//Store the equivalence between neighboring labels
				if(top_number_label != 0 && left_number_label != 0 && (top_number_label != left_number_label))
				{//when the labels are different and not 0, we have to store the equivalence
					disjointSet.AddElement(top_number_label, left_number_label);
				}
			}
		}
	}//for
	disjointSet.Equalize(); //calculate smallest values which equal labelled clusters should be assigned to
		//SECOND PASS
	for(int i = 0; i < mask->height; i++){
		for(int j = 0; j < mask->width; j++){
			if(ClusterMap[i][j] != 0){
				ClusterMap[i][j] = disjointSet.GetSmallestElementInSet(ClusterMap[i][j]);
			}
		}
	}
	//END ALGORITHM

	//now we have cluster_counter amount of potential clusters (actual is smaller since lots now have 0 size) but it's a strict upper boundry
	clusterNumber = cluster_counter;
	number_of_white_pixels = white_pixel_counter;
	
	return;
}

void ImageCalculation:: DetermineWhitePixelCoords() //make internal
{
	if(whitePixelInformationSize < number_of_white_pixels){
		InitializeWPI(number_of_white_pixels);
	}
	
	BwImage enter(mask);
	int counter = 0;
	for(int i = 0; i < mask->height; i++){
		for(int j = 0; j < mask->width; j++){
			if(enter[i][j] != 0){ 
				whitePixelInformation[_X_COORD][counter] = j;
				whitePixelInformation[_Y_COORD][counter] = i;
				whitePixelInformation[_CLUSTER_NUMBER][counter] = ClusterMap[i][j]; //Initialize this array value as 0 at the same time... only use this with non-recursive algorithm
				counter++;
			}
		}
	}
	return;
}

void ImageCalculation:: DetermineClusterSizes()
{
	//enlarge the array if it's not large enough
	if(clusterSizesSize < clusterNumber){
		InitializeClusterSizes(clusterNumber);
	}
	//zero clusterSizes, resetting the counts
	for(int i = 0; i < clusterNumber; i++){
		clusterSizes[i] = 0;
	}

	//add clusterarray[3][i] (pixel with certain clusternumber) to the clusterSizes
	for(int i = 0; i < number_of_white_pixels; i++){
		clusterSizes[whitePixelInformation[_CLUSTER_NUMBER][i]]++;
	}
	return;
}

void ImageCalculation:: FindNBiggestClusters(const int N)
{
	//of the largestClusterNumber array is too small, reinitialize it
	if(largestClusterNumbersSize < N){
		InitializelargestClusterNumbers(N);
	}

	unsigned int largest; //store the largest value till now
	int index; //and its index
	for(int i = 0; i < N; i++){//find the N largest clusters
		largest = 0;
		index = 0;
		for(int j = 0; j < clusterNumber; j++){
			if(largest < clusterSizes[j]){
				largest = clusterSizes[j];
				index = j;
			}
		}
		largestClusterNumbers[i] = index;
		largestClusterNumbersSizes[i] = clusterSizes[index];
		clusterSizes[index] = 0; //set to 0.
	}
	return;
}

void ImageCalculation:: DetermineNMeans(const int N){
	if(meansSize < N){ //reinitialize if the array is too small
		InitializeMeans(N);
	}

	unsigned int *smallestx = new unsigned int[N]; //arrays to store largest and smallest values in
	unsigned int *smallesty = new unsigned int[N];
	unsigned int *largestx = new unsigned int[N];
	unsigned int *largesty = new unsigned int[N];
	for(int i = 0; i < N; i++){ //initialize on impossible maxima and minima
		smallestx[i] = mask->width*2;
		smallesty[i] = mask->height*2;
		largestx[i] = 0;
		largesty[i] = 0;
	}

	for(int i = 0; i < number_of_white_pixels; i++){ //for every pixel, we check it's cluster number
		for(int j = 0; j < N; j++){ //for each cluster number
			if(largestClusterNumbers[j] == whitePixelInformation[_CLUSTER_NUMBER][i]){ //if this pixel belongs to this cluster
				if(whitePixelInformation[_X_COORD][i] < smallestx[j]){ //check if its x coordinate is the smallest found till now
					smallestx[j] = whitePixelInformation[_X_COORD][i];
				}
				if(whitePixelInformation[_X_COORD][i] > largestx[j]){//do the same for all other coordinates
					largestx[j] = whitePixelInformation[_X_COORD][i];
				}
				if(whitePixelInformation[_Y_COORD][i] < smallesty[j]){
					smallesty[j] = whitePixelInformation[_Y_COORD][i];
				}
				if(whitePixelInformation[_Y_COORD][i] > largesty[j]){
					largesty[j] = whitePixelInformation[_Y_COORD][i];
				}
			}
		}
	}

	if(regiosize < N){
		InitializeRegions(N);
	}

	for(int i = 0; i < N; i++){ //we now calculate the means
		if(largestClusterNumbersSizes[i] > _PIXEL_LIMIT){
			means[_X_COORD][i] = (smallestx[i] + largestx[i])/2; //averages of the smallest and largest pixel
			means[_Y_COORD][i] = (smallesty[i] + largesty[i])/2;
			regions[0][i] = (largestx[i] - smallestx[i]);
			regions[1][i] = (largesty[i] - smallesty[i]);
		}else{
			means[_X_COORD][i] = 0; //we return 0 if there are not enough pixels in the clusters
			means[_Y_COORD][i] = 0; //this means that this probably is not a real significant cluster
			regions[0][i] = 0;
			regions[1][i] = 0;
		}
	}

	delete [] smallestx;
	delete [] smallesty;
	delete [] largestx;
	delete [] largesty;

	return;
}

int ImageCalculation::getmean(int clusternum, int xory){
	if(clusternum >= meansSize){
		return 0;
	}
	return static_cast<int>(means[xory][clusternum]);
}

void ImageCalculation::InitializeRegions(const int N){
	DestroyRegions();
	regions = new int * [2];
	regions[0] = new int [N];
	regions[1] = new int [N];
	regiosize = N;
}

void ImageCalculation::DestroyRegions(){
	if(regions != NULL){
		if(regions[0] != NULL){
			delete [] regions[0];
			regions[0] = NULL;
		}
		if(regions[1] != NULL){
			delete [] regions[1];
			regions[1] = NULL;
		}
		delete [] regions;
		regions = NULL;
	}
}


int ImageCalculation::getRegions(int clusternum, int xory){
	if(clusternum >= regiosize){
		return 0;
	}
	return static_cast<int>(regions[xory][clusternum]);
}