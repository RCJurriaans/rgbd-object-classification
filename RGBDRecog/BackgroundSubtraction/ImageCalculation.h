#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Model of a Color is a CvHistogram
typedef CvHistogram *ColorModel;

class ImageCalculation
{
public:
	ImageCalculation(void);
	~ImageCalculation(void);

	//functions
	void Calculate(IplImage* thisImage, int numberOfMarkers);
	int getmean(int clusternum, int xory);
	int getRegions(int clusternum, int xory);

private:

	//Functions
	void InitializeClusterMap(int height, int width); //Initialize ClusterMap with size height and width. Assign ClusterMapSize those parameters;
	void DestroyClusterMap(); //delete the full ClusterMap of size ClusterMapSize
	
	void InitializeWPI(int whitePixelAmount); //initializes the whitePixelInformation array on [3][whitePixelAmount]
	void DestroyWPI(); //deletes the whole array
	
	void InitializeClusterSizes(int numberofclusters);
	void DetroyClusterSizes();

	void InitializelargestClusterNumbers(const int N);
	void DestroylargestClusterNumbers();

	void InitializeMeans(const int N);
	void DestroyMeans();

	void InitializeRegions(const int N);
	void DestroyRegions();

	//Applies a 4 connectivity connected component algorithm on the mask.
	//Stores the labels in ClusterMap. Every pixel in ClusterMap is connected to another pixel
	// if their labels have the same number
	// stores the number of nonblack pixels in number_of_white_pixels and an upper bound for the labels of the clusters in clusterNumber
	void DetermineClusterNumbers();

	//saves the coordinates and clusternumber of each white pixel in whitepixelinformation[][];
	//call this after DetermineClusterNumbers.
	// _X_COORD in the first parameter gives the x coord of the pixel in the second parameter
	// same goes for _Y_COORD and _CLUSTER_NUMBER
	void  DetermineWhitePixelCoords();
	
	//calculates the sizes of all clusters with a certain number. Store these sizes in clusterSizes, with the 
	//number of the cluster (label) as a parameter;
	void DetermineClusterSizes();

	//uses clusterSizes to determine the N largest clusters. Puts the index in largestClusterNumbers. puts the size
	// in largestClusterNumbersSizes
	//do not use clusterSizes after this, it will have the largest values set to 0
	void FindNBiggestClusters(int N);


	//uses largestClusterNumbers, largestClusterNumberSizes, and whitePixelInformation
	// to calculate the means of the largest clusters, and stores this in means
	// means can aftewards be called with means[_X_COORD/_Y_COORD][#cluster]
	void DetermineNMeans(const int N);

	//Variables
	IplImage * mask; // area used for calculation || init on NULL || deleted
	
	unsigned int ** ClusterMap; //used to store cluster numbers in ||always has at least the same size as mask|| first init on NULL || init in Calculate || deleted;
	CvSize ClusterMapSize; //used to store the size of ClusterMap for Deletion || use the mask sizes for more efficient computation || changed in size when it's smaller than the gotten map, in DetermineClusterNumbers()
	
	unsigned int ** whitePixelInformation; //contains the coords and cluster numbers of all white pixels || init on NULL || deleted
	int whitePixelInformationSize; //current size of the second dimension of the whitePixelInformation array || init on 0 || adjusted in DetermineWhitePixelCoords || deleted
	
	unsigned int * clusterSizes; //used to store the sizes of the clusters with a number (which is given as a paramter) || init on 0 || adjusted in DetermineClusterSizes || deleted
	int clusterSizesSize; //current size of the clusterSizes array; || init on 0

	unsigned int * largestClusterNumbers; //used to store the numbers of the N largest clusters found, where N is a passed parameter || init on 0 || adjusted in FindNBiggestClusters  || deleted
	unsigned int * largestClusterNumbersSizes; //used to store the actual pixel sizes of these clusters || init on 0 || adjusted in FindNBiggestClusters  || deleted
	int largestClusterNumbersSize; //size of this array || init on 0

	unsigned int ** means; //used to store the N means (x = 0, y = 1 first param) || init on NULL || Adjusted in DetermineNMeans || Deleted
	int meansSize; //size of this array || init on 0

	int ** regions; //Used to store the sizes of the clusters in x and y direction || init on NULL || adjusted in DetermineNMeans || Deleted
	int regiosize; // size of this array || init on 0

	int clusterNumber; //upper bound for the number of clusters there are  || Only use after calling DetermineClusterNumbers() || init on 0
	int number_of_white_pixels; //amount of white pixels found in the image || Only use after calling DetermineClusterNumbers() || init on 0
};

