

#include "stdafx.h"
#include "NNMenu.h"
#include "RFClass.h"

NNMenu::NNMenu(Settings * set){
	nbnn = new NBNN;

	settings = set;

	rng = new cv::RNG(1337);

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

	featureExtractor = new FeatureExtractor();

	for(int i = 0; i < amountOfClasses; i++){
		int picNumSum = testPicNum[i] + trainigPicNum[i];
		testdataID.push_back(DrawWithReplacementNN(picNumSum,testPicNum[i]));
		vector<int> temptrainingIDs;
		for(int j = 1; j <= picNumSum; j++){
			bool found = false;
			for(unsigned int k = 0; k < testdataID[i].size(); k++){
				if(testdataID[i][k] == j){
					found = true;
				}
			}
			if(!found){
				temptrainingIDs.push_back(j);
			}
		}
		for (int j=1; j<(trainigPicNum[i]); j++) { //randomly shuffle
            int r = j + (rand() % (trainigPicNum[i]-j)); // Random remaining position.
            int temp = temptrainingIDs[j]; temptrainingIDs[j] = temptrainingIDs[r]; temptrainingIDs[r] = temp;
        }
		trainingdataID.push_back(temptrainingIDs);
	}
}

vector<int> NNMenu:: DrawWithReplacementNN(int maxrange, int amount){
	vector<int> returnvalues;
	if(maxrange <= amount){
		return returnvalues; //makes certain that the function stops
	}

	while(returnvalues.size() < amount){
		int drawnValue = rng->operator()(maxrange);
		bool found = false;
		for(unsigned int j = 0; j < returnvalues.size(); j++){
			if(returnvalues[j] == drawnValue){
				found = true;
			}
		}
		if(!found){
			returnvalues.push_back(drawnValue);
		}
	}
	for(unsigned int i = 0; i < returnvalues.size(); i++){
		returnvalues[i] ++;
	}
	return returnvalues;
}

void NNMenu::menu(){
	char option = ' ';
	while(option){
		cout << "Choose an option: " << endl
			 << "  (T) raining the model" << endl
			 << "  (A) pply model to test set" << endl
			 << "  (Q) uit" << endl ;
		cin.clear();
		cin.sync();
		option = cin.get();
		switch(option){
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
			} break;
			case 't': case 'T':{
				cin.clear();
				cin.sync();
				cout << "Loading training" << endl;
				trainData();
			} break;
			case 'a': case 'A':{
				cin.clear();
				cin.sync();
				cout << "Running testing" << endl;
				NNTesting();
			} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}//switch
	}//while
}

NNMenu::~NNMenu(void){
	delete nbnn;
}

cv::Rect NNMenu::readRectNN(const string filePath){
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

cv::Rect NNMenu:: getDatasetROINN(string folderPath, int j){
	string rectPath = folderPath;
	rectPath += "imgRECT" + RFClass::convertNumberToFLString(3,j) + ".txt";
	cv::Rect roi = readRectNN(rectPath);
	return roi;
}

void NNMenu::trainData()
{
	cout << "Creating training data for NN. Settings: " << endl;
	settings->outputSettings();
	cout << endl;
	cout << "parameters are from data_info.txt:" << endl;
	cout << "Initializing training creation files..." << endl;
	cout << "Assuming that the training data files are located in: " << getenv("RGBDDATA_DIR") << endl;

	string imagePath; //path where the images are
	string filePath; //filepath for each image
	string pcdPath; //for the pcd data
	string maskOutPath;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Mat input; //stores the image data

	for(int i = 0; i < amountOfClasses; i++){

		vector<cv::Mat> featurevectors; // Contains a mat for each image

		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << endl;
		for(int j = 0; j < trainigPicNum[i]; j++){ //for each image
			int chosenImgID = trainingdataID[i][j];
			imagePath.clear();
			if(!settings->segmentation){
				imagePath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + fileExtension; //get the proper filename
				pcdPath = filePath + "img" +RFClass:: convertNumberToFLString(3,chosenImgID) + ".pcd";
				maskOutPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "mask" + fileExtension;
			}else{
				imagePath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "seg" + fileExtension; //get the proper filename
				pcdPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "seg" + ".pcd";
				maskOutPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "mask" + fileExtension;
			}
			pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdPath, *cloud);

			cout << "processing on image: " << classNames[i] << "_" << chosenImgID;
			input.release();
			input = cv::imread(imagePath); //Load as grayscale image
			cv::Mat mask = cv::imread(maskOutPath);
			bool found = false;

			vector<cv::Mat> tempfeaturevector;
			if(settings->segmentation){
				//tempfeaturevector.clear();
				cv::Rect roi = getDatasetROINN(filePath,chosenImgID);
				if(roi.width*roi.height > 0){
					//tempfeaturevector = featureExtractor->extractFeatures(settings->modes,input,roi);
					tempfeaturevector = featureExtractor->extractRawFeatures(settings->modes,input,cloud,mask);
					found = true;
				}else{
					found = false;
				}
			}else{
				if(input.cols > 0){
					//tempfeaturevector.clear();
					tempfeaturevector = featureExtractor->extractRawFeatures(settings->modes,input,cloud,mask);
					found = true;
				}else{
					found = false;
				}
			}
			
			//tempfeaturevector now contains all found feature descriptors,
			//we can add this to the neirest neihbour classifier
			if(found && tempfeaturevector[0].cols > 0){
				featurevectors.push_back( tempfeaturevector[0] );

				/*if(featurevector[0].cols == 0){
					featurevector[0].release();
					featurevector[0] = tempfeaturevector[0];
				}else{
					hconcat(featurevector[0],tempfeaturevector[0],featurevector[0]);
				*/
				cout <<  " using image" << endl;
			}else{
				cout << " image discarded" << endl;
			}
		}
		nbnn->addInstancesToClass(featurevectors);//,i);
		//featurevector.clear();
	}

	string savePath = getenv("RGBDDATA_DIR");
	savePath += "\\NBNN" + settings->settingsString() + ".yml";
	cout << "Writing NN data to: " << savePath << endl;
	cv::FileStorage f(savePath, cv::FileStorage::WRITE);
	nbnn->write(f);
	f.release();
	cout << "Done writing NN." << endl;
}

void NNMenu::NNTesting(){

	if(!featureExtractor->codeBooksLoaded()){
		featureExtractor->loadCodebooks();
	}

	cout << "Starting testing the random forest algorithm on all test data" << endl;
	cout << "Using settings: " << endl;
	settings->outputSettings();
	//create a string of the modeboolean
	string modestring = settings->settingsString();
	//read the data of the random forest
	//rfclassifier->read("randomforestdata" + modestring + ".yml", "randomforestdata" + modestring);
	//now we use the random forest to predict the class of each of the test images

	cv::Mat input;
	string imagePath; //path where the images are
	string filePath; //filepath for each image
	string pcdPath; //for the pcd data
	string maskOutPath;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	vector<cv::Mat> result;
	int** predictions = new int*[amountOfClasses];
	for(int i = 0; i < amountOfClasses; i++){
		predictions[i] = new int [testPicNum[i]+1];
	}


	string savePath = getenv("RGBDDATA_DIR");
	savePath += "\\NBNN" + settings->settingsString() + ".yml";
	cout << "Reading NN data from: " << savePath << endl;
	cv::FileStorage f(savePath, cv::FileStorage::READ);
	nbnn->read(f);
	f.release();

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "\\"; //go to the classname folder
		cout << "procesing class: " << classNames[i] << endl;
		for(int j = 0; j < testPicNum[i]; j++){ //for each image
			int chosenImgID = testdataID[i][j];
			imagePath.clear();
			if(!settings->segmentation){
				imagePath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + fileExtension; //get the proper filename
				pcdPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + ".pcd";
				maskOutPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "mask" + fileExtension;

			}else{
				imagePath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "seg" + fileExtension; //get the proper filename
				pcdPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "seg" + ".pcd";
				maskOutPath = filePath + "img" + RFClass::convertNumberToFLString(3,chosenImgID) + "mask" + fileExtension;
			}
			pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdPath, *cloud);
			cout << "processing image: " << classNames[i] << "_" << chosenImgID;
			input.release();
			input = cv::imread(imagePath); //Load rgb image
			result.clear();
			cv::Mat mask = cv::imread(maskOutPath);
			bool found = false;
			if(settings->segmentation){
				cv::Rect roi = getDatasetROINN(filePath,chosenImgID);
				if(roi.width*roi.height > 0){
					//result = featureExtractor->extractFeatures(settings->modes,input,roi);
					result = featureExtractor->extractRawFeatures(settings->modes,input,cloud,mask);
					found = true;
				}else{
					found = false;
				}
			}else{
				if(input.cols > 0){
					result = featureExtractor->extractRawFeatures(settings->modes,input,cloud,mask);
					found = true;
				}
				else{
					found = false;
				}
			}
			if(found && result[0].cols > 0){
				//predictions[i][j] = rfclassifier->predict(result);
				predictions[i][j] = nbnn->classify(result[0]);
				cout << " classified as: " << classNames[predictions[i][j]] << endl;
			}else{
				predictions[i][j] = rng->operator()(featureExtractor->getAmountOfFeatures());
				cout << " predicting randomly" << endl;
			}
		}
	}


	cout << "Classified all images, calculating the accuracy" << endl;
	vector<float> accuracy;
	for(int i = 0; i < amountOfClasses; i++){
		float sum = 0;
		for(int j = 1; j <= testPicNum[i]; j++){
			if(predictions[i][j] == i){
				sum++;
			}
		}
		sum = sum/testPicNum[i];
		accuracy.push_back(sum);
	}

	cout << "Displaying accuracy results:" << endl;
	for(int i = 0; i < amountOfClasses; i++){
		cout << classNames[i] << ": " << accuracy[i] << endl;
	}
	float accuracysum = 0;
	for(int i = 0; i < amountOfClasses; i++){
		accuracysum+= accuracy[i];
	}
	accuracysum = accuracysum / amountOfClasses;
	cout << "Average accuracy: " << accuracysum << endl;


	for(int i = 0; i < amountOfClasses; i++){
		delete [] predictions[i];
	}
	delete [] predictions;
}