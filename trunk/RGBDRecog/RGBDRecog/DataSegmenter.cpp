#include "StdAfx.h"
#include "DataSegmenter.h"

using namespace std;

string DataSegmenter:: convertNumberToFLString(int length, int number){
	int tempnum = number;
	string returnvalue;
	
	length--;
	
		while(tempnum/10 != 0){
			tempnum = tempnum/10;
			length--;
		}
		for(int k = 0; k < length; k++){
			returnvalue += "0";
		}
		returnvalue += boost::lexical_cast<string>(number);
	return returnvalue;
}

DataSegmenter::DataSegmenter(void)
{
	ifstream infile;
	string filename;
	filename += getenv("RGBDDATA_DIR") ;
	filename += "\\data_info.txt";
 	infile.open(filename); //open data_info.txt etc.
	if(!infile.good()){
		cout << "Could not find %RGBDATA_DIR%/data_info.txt" << endl;
		return;
	}
	string temps; //temporary read variables

	char temp[256];
	int tempi;
	infile.getline(temp,256);
	infile >> amountOfClasses; //amount of classes in dataset

	infile >> fileExtension;

	 //get the names of the classes
	for(int i = 0; i < amountOfClasses; i++){
		infile >> temps;
		classNames.push_back(temps);
	}
	infile.ignore('\n');
	infile.getline(temp,256);
	//get the amount of images for each training set class
	for(int i = 0; i < amountOfClasses;i++){ 
		infile >> tempi;
		trainigPicNum.push_back(tempi);
	}
	infile.ignore('\n');
	infile.getline(temp,256);
	//get the amount of images in the training set.
	for(int i = 0; i < amountOfClasses;i++){ 
		infile >> tempi;
		testPicNum.push_back(tempi);
	}

	//output results
	cout << endl;
	cout << "Loaded dataset parameters:" << endl << "Class names | training images | test images:" << endl;
	for(int i = 0; i < amountOfClasses; i++){
		cout << "  " << classNames[i] << ' ' << trainigPicNum[i] << ' ' << testPicNum[i] << endl;
	}
	infile.close();

	segmentation = new SegmentCloud();
}


DataSegmenter::~DataSegmenter(void)
{
	delete segmentation;
}

void DataSegmenter::writeRect(const string filePath,const cv::Rect rect){
	ofstream out;
	out.open(filePath);

	out << rect.x << endl;
	out << rect.y << endl;
	out << rect.width << endl;
	out << rect.height << endl;

	out.close();
}

cv::Rect DataSegmenter::readRect(const string filePath){
	ifstream in;
	in.open(filePath);

	cv::Rect rect;

	in >> rect.x;
	in >> rect.y;
	in >> rect.width;
	in >> rect.height;

	in.close();
	return rect;
}

void DataSegmenter::generateBoundingBoxes(){
	string filePath;	
	string imagePath;
	string pcdPath;
	string outputPath;
	string outputImg;
	string pcdOutPath;
	string maskOutPath;
	cv::Mat input;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr background (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	string backgroundPath;



	//set the proper segmentation method to background subtraction
	segmentation->setSegMethod(segmentation->SegBack); //set to back projection
	

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << "_train" << endl;

		for(int j = 1; j <= trainigPicNum[i] + testPicNum[i]; j++){ //for each image
			imagePath.clear();
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename
			pcdPath = filePath + "img" + convertNumberToFLString(3,j) + ".pcd";
			outputPath = filePath + "imgRECT" + convertNumberToFLString(3,j) + ".txt";
			outputImg = filePath + "img" + convertNumberToFLString(3,j) + "seg" + fileExtension;
			backgroundPath = filePath + "bg" + convertNumberToFLString(3,j) + ".pcd";
			pcdOutPath = filePath + "img" + convertNumberToFLString(3,j) + "seg" + ".pcd";
			maskOutPath = filePath + "img" + convertNumberToFLString(3,j) + "mask" + fileExtension;

			//load the background data
			pcl::io::loadPCDFile<pcl::PointXYZRGB> (backgroundPath, *background);

			input.release();
			input = cv::imread(imagePath);

			cout << "processing on image: " << classNames[i] << "_" << j << endl;

			pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdPath, *cloud);

			//get region of interest from the cloud data
			cv::Rect rect; // = segmentation->getROI(cloud, background);
			cv::Rect expectedROI = segmentation->getROI(cloud, background);
			boost::shared_ptr<cv::Mat> mask;
			segmentation->setBackground(background);
			segmentedCloud = segmentation->tijmenLikesHacking(mask, rect,cloud);

			//write the rectangle to a file
			writeRect(outputPath,rect);
			if(rect.width > 0 && rect.height > 0){
				cv::imwrite(outputImg, input(rect));
			}
			//write the mask away
			imwrite(maskOutPath,*mask);

			//write the segmented point cloud to a file too
			pcl::io::savePCDFile<pcl::PointXYZRGB> (pcdOutPath,*segmentedCloud); //*segmentation->getWindowCloud(cloud,background));


		}
	}
}

void DataSegmenter::menu(){
	char option = ' ';

	while(option != 'q' && option != 'Q'){
		cout << "Choose an option to apply to the dataset: " << endl
			 << "  (B) ounding box creation" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'b': case 'B':{
				cin.clear();
				cin.sync();
				cout << "Running bounding box creation on dataset." << endl;
				generateBoundingBoxes();
			} break;
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
			} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			} break;
		}
	}
}
