#include "StdAfx.h"
#include "RFClass.h"
#include "FeatureVector.h"
#include <fstream>
#include <time.h>

#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>

//using namespace cv;
using namespace std;



RFClass::RFClass(void){

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

	infile.open("RFParams.txt");
	if(!infile.good()){
		cout << "Could not find RFParams.txt" << endl;
		return;
	}
	while(infile.get() != ' '){ //ignore the first word parts
	}
	infile >> SIFTThreshScale; //get this parameter
	while(infile.get() != ' '){//ignore the first word parts
	}
	infile >> dicsize;
	infile.close();
	cout << "Read RF parameters: " << endl;
	cout << "  " << "Sift Treshold Scale: " << SIFTThreshScale << endl;
	cout << "  " << "Codebook size: " << dicsize << endl;
	cout << endl;

	featureExtractor = new FeatureExtractor();
	rfclassifier = new RFClassifier();
}



RFClass::~RFClass(void){
	delete featureExtractor;
	delete rfclassifier;
}

//converts a number to a fixed length string, adding zeros in front
string RFClass:: convertNumberToFLString(int length, int number){
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

void RFClass::createCodebook(int mode){
	cout << "Running codebook creation with setting: " << featureExtractor->getFeatureName(mode) << endl;
	cout << "parameters are from data_info.txt:" << endl;
	cout << "  Dictionary size: " << dicsize << endl;
	cout << "  SIFT Threshold Scale: " << SIFTThreshScale << endl;
	cout << "Initializing codebook creation files..." << endl;
	cout << "Assuming that the training data files are located in: " << getenv("RGBDDATA_DIR") << endl;

	clock_t init, final; //for timing
	init=clock();

	string imagePath; //path where the images are
	string filePath; //filepath for each image
	cv::Mat input; //stores the image data
	FeatureVector* fv = new FeatureVector; //this contains all features that are extacted

	cout << "Running image feature extraction" << endl;
	cout << "Using the first half of the training images to create the codebooks" << endl;

	//construct a boolean vector with the single mode in modes,
	// used as a parameter for the featureextraction
	vector<bool> modes;
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		if(i == mode){
			modes.push_back(true);
		}else{
			modes.push_back(false);
		}
	}

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << endl;
		for(int j = 1; j <= floor(static_cast<double>(trainigPicNum[i])/2); j++){ //for each image

			imagePath.clear();
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename

			cout << "processing on image: " << classNames[i] << "_" << j << endl;
			input.release();
			input = cv::imread(imagePath);
			//this should return a single cv::Mat with the descriptor, as only one value in modes is true
			//features are in the rows



			vector<cv::Mat> RawFeatures = featureExtractor->extractRawFeatures(modes,input);


			fv->AddFeatures(RawFeatures[0]);
			RawFeatures[0].release();
			RawFeatures.clear();
			input.release();
		}
	}
	cout << "found " << fv->features->rows << " descriptors."<< endl;
	cout << "Running kmeans" << endl;
	cv::Mat* codebook = fv->kmeans(dicsize); //run kmeans on the data for dicsize clusters
	cout << "writing the result to file: " << "codebook" + boost::lexical_cast<string>(mode) + ".yml" << endl;
	cv::FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", cv::FileStorage::WRITE); //store the codebook to a yaml file
	fs << "codebook" + boost::lexical_cast<string>(mode) << *codebook;
	final=clock()-init;
	
	cout <<  "Done creating the " << featureExtractor->getFeatureName(mode) << " codebook in "  << (double)final / ((double)CLOCKS_PER_SEC*60) << " minutes" <<endl;
	delete fv;
	codebook->release();
	delete codebook;
}

string RFClass::ifBoolReturnChar(bool in, string out){
	if(in){
		return out;
	}
	else return " ";
}

void RFClass::createCodebookMenu(){
	char option = ' ';
	vector<bool> foundCodebooks;
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		cv::FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
		if(fs.isOpened()){
			foundCodebooks.push_back(true);
		}
		else{
			foundCodebooks.push_back(false);
		}
	}
	while(option){
		cout << "For which feature type do you want to create the codebook? " << endl
			 << "  (S) tandard SIFT " << ifBoolReturnChar(foundCodebooks[0],"existing") <<endl
			 << "  (H) ue SIFT " << ifBoolReturnChar(foundCodebooks[1],"existing") << endl
			 << "  (O) pponent SIFT " << ifBoolReturnChar(foundCodebooks[2],"existing") << endl
			 << "  (1) Normal SURF " << ifBoolReturnChar(foundCodebooks[3],"existing") << endl
			 << "  (2) Hue SURF " << ifBoolReturnChar(foundCodebooks[4],"existing") << endl
			 << "  (3) Opponent SURF " << ifBoolReturnChar(foundCodebooks[5],"existing") << endl
			 << "  (A) ll" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
		} break;
			case 's':  case 'S':{
				cin.clear();
				cin.sync();
				cout << "Creating standard SIFT codebook" << endl;
				createCodebook(0);
		} break;
			case 'h': case 'H':{
				cin.clear();
				cin.sync();
				cout << "Creating Hue SIFT codebook" << endl;
				createCodebook(1);
		} break;
			case 'o': case 'O':{
				cin.clear();
				cin.sync();
				cout << "Creating Opponent SIFT codebook" << endl;
				createCodebook(2);
		} break;
			case 'a': case 'A':{
				cin.clear();
				cin.sync();
				cout << "Creating all codebooks" << endl;
				createCodebook(0);
				createCodebook(1);
				createCodebook(2);
				createCodebook(3);
				createCodebook(4);
				createCodebook(5);
		} break;
			case '1': {
				cin.clear();
				cin.sync();
				cout << "Creating Normal SURF codebook" << endl;
				createCodebook(3);
		} break;
			case '2': {
				cin.clear();
				cin.sync();
				cout << "Creating Hue SURF codebook" << endl;
				createCodebook(4);
		} break;
			case '3': {
				cin.clear();
				cin.sync();
				cout << "Creating Opponent SURF codebook" << endl;
				createCodebook(5);
		} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}

void RFClass::addDescriptor(bool & firstadded, cv::Mat & tempfeaturevector, FeatureVector * codebooks,cv::Mat & descriptors, int mode){
	if(!firstadded){
		tempfeaturevector.release();
		tempfeaturevector = codebooks[mode].GenHistogram(descriptors);
		firstadded = true;
	}else{
		vconcat(tempfeaturevector,codebooks[mode].GenHistogram(descriptors),tempfeaturevector);
	}
}

void RFClass:: trainModel(vector<int> mode){
	vector<bool> modes;
	//const char *_COLORNAMES[] = {"NORMALSIFT", "HUESIFT", "OPPONENTSIFT","NORMALSURF", "HUESURF", "OPPONENTSURF"};
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		modes.push_back(false);
	}
	for(unsigned int i = 0; i < mode.size(); i++){ //set the options of the algorithm
		modes[mode[i]] = true;
	}
	
	cout << "Running training with settings: ";
	for(unsigned int i = 0; i < modes.size(); i++){
		if(modes[i]){
			cout << featureExtractor->getFeatureName(i) << " ";
		}
	}
	cout << "parameters are from data_info.txt:" << endl;
	cout << "  Dictionary size: " << dicsize << endl;
	cout << "  SIFT Threshold Scale: " << SIFTThreshScale << endl;
	cout << "Initializing training creation files..." << endl;
	cout << "Assuming that the training data files are located in: " << getenv("RGBDDATA_DIR") << endl;

	FeatureVector *codebooks;
	codebooks = new FeatureVector[featureExtractor->getAmountOfFeatures()];

	//load the codebook data
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		if(modes[i] == true){
			cv::FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
			cv::Mat M; fs["codebook" + boost::lexical_cast<string>(i)] >> M;
			fs.release();
			codebooks[i].AddFeatures(M);
			codebooks[i].TrainkNN();
		}
	}

	string imagePath; //path where the images are
	string filePath; //filepath for each image
	cv::Mat input; //stores the image data

	cv::Mat tempfeaturevector;
	vector<cv::Mat> featurevector;

	//load the featureExtractor codebooks if they are not yet loaded
	if(!featureExtractor->codeBooksLoaded()){
		featureExtractor->loadCodebooks();
	}

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << endl;
		for(int j = static_cast<int>(floor(static_cast<double>(trainigPicNum[i])/2))+1; j <= trainigPicNum[i]; j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename
			cout << "processing on image: " << classNames[i] << "_" << j << endl;
			input.release();
			input = cv::imread(imagePath); //Load as grayscale image
			
			cv::Mat tempfeaturevector = featureExtractor->extractFeatures(modes,input);
			
			//tempfeaturevector now contains all found feature descriptors,
			//we can add this to this class

			if(j == floor(static_cast<double>(trainigPicNum[i])/2)+1){
				featurevector.push_back(tempfeaturevector);
			}else{
				hconcat(featurevector[i],tempfeaturevector,featurevector[i]);
			}
		}
	}


	string modestring;
	modestring.clear();
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		if(modes[i]){
			modestring += '1';
		}
		else{
			modestring += '0';
		}
	}

	cout << "Done creating the feature vectors for the training set" << endl
		 << "Saving the data to " << "trainingdata" + modestring + "RF" + ".yml";



	cv::FileStorage fs2("trainingdata" + modestring + "RF" ".yml", cv::FileStorage::WRITE); //store the codebook to a yaml file
	for(int i = 0; i < amountOfClasses;i++){
		fs2 << "trainingdata" + modestring + boost::lexical_cast<string>(i) << featurevector[i];
	}
	fs2.release();

	cout << "Saved trainingdata" << endl;



}

void RFClass::trainModelMenu(){
		char option = ' ';
	vector<bool> foundCodebooks;
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		cv::FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", cv::FileStorage::READ);
		if(fs.isOpened()){
			foundCodebooks.push_back(true);
		}
		else{
			foundCodebooks.push_back(false);
		}
	}

	vector<int> chosenOptions;
	while(option){
		cout << "For which feature type do you want to train the data? (multiple possible)" << endl << endl
			 << "Chosen features: ";
		for(unsigned int i = 0; i < chosenOptions.size(); i++){
			cout << featureExtractor->getFeatureName(chosenOptions[i]) << " ";
		}
		cout << endl
		     << "  (S) Standard SIFT " << ifBoolReturnChar(foundCodebooks[0],"codebook exists ") << endl
			 << "  (H) Hue SIFT " << ifBoolReturnChar(foundCodebooks[1],"codebook exists ") <<  endl
			 << "  (O) Opponent SIFT " << ifBoolReturnChar(foundCodebooks[2],"codebook exists ") <<  endl
			 << "  (1) Normal SURF " << ifBoolReturnChar(foundCodebooks[3],"codebook exists ") << endl
			 << "  (2) Hue SURF " << ifBoolReturnChar(foundCodebooks[4],"codebook exists ") << endl
			 << "  (3) Opponent SURF " << ifBoolReturnChar(foundCodebooks[5],"codebook exists ") <<  endl
			 << "  (T) rain with the current settings" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
		} break;
			case 's':  case 'S':{
				cin.clear();
				cin.sync();
				cout << "Adding standard SIFT to used features" << endl;
				chosenOptions.push_back(0);
		} break;
			case 'h': case 'H':{
				cin.clear();
				cin.sync();
				cout << "Adding hue SIFT to used features" << endl;
				chosenOptions.push_back(1);
		} break;
			case 'o': case 'O':{
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(2);
		} break;
			case '1': {
				cin.clear();
				cin.sync();
				cout << "Adding normal SURF to used features" << endl;
				chosenOptions.push_back(3);
		} break;
			case '2': {
				cin.clear();
				cin.sync();
				cout << "Adding hue SURF to used features" << endl;
				chosenOptions.push_back(4);
		} break;
			case '3': {
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(5);
		} break;
			case 't': case 'T': {
				cin.clear();
				cin.sync();
				cout << "Starting training with chosen features" << endl;
				trainModel(chosenOptions);
		} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}

void RFClass::generateRandomForest(vector<int> mode){
	vector<bool> modes;//create boolean array of modes
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		modes.push_back(false);
	}
	for(unsigned int i = 0; i < mode.size(); i++){ //set the options of the algorithm
		modes[mode[i]] = true;
	}
	
	//create a string of the modeboolean
	string modestring;
	modestring.clear();
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		if(modes[i]){
			modestring += '1';
		}
		else{
			modestring += '0';
		}
	}

	vector<cv::Mat> trainingdata;
	cv::FileStorage fs("trainingdata" + modestring +"RF" +  ".yml", cv::FileStorage::READ);
	cv::Mat temp;
	for(int i = 0; i < amountOfClasses; i++){
		temp.release();
		fs["trainingdata" + modestring+ boost::lexical_cast<string>(i)] >> temp;
		trainingdata.push_back(temp);
	}
	fs.release();

	cout << "Starting training of random forest" << endl;
	rfclassifier->trainTree(trainingdata);

	cout << "Starting output of random forest data" << endl;
	rfclassifier->write("randomforestdata" + modestring + ".yml", "randomforestdata" + modestring);
	
	cout << "Done with the random forest creation and saving" << endl;
}

void RFClass::rfTrainigmenu(){
	char option = ' ';

	vector<int> chosenOptions;
	while(option){
		cout << "For which feature type do you want to generate the random forest? (multiple possible)" << endl << endl
			 << "Chosen features: ";
		for(unsigned int i = 0; i < chosenOptions.size(); i++){
			cout << featureExtractor->getFeatureName(chosenOptions[i]) << " ";
		}
		cout << endl
		     << "  (S) Standard SIFT " << endl
			 << "  (H) Hue SIFT " <<  endl
			 << "  (O) Opponent SIFT "<<  endl
			 << "  (1) Normal SURF "  << endl
			 << "  (2) Hue SURF " <<  endl
			 << "  (3) Opponent SURF " <<  endl
			 << "  (R) andom forest creation with the current settings" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
		} break;
			case 's':  case 'S':{
				cin.clear();
				cin.sync();
				cout << "Adding standard SIFT to used features" << endl;
				chosenOptions.push_back(0);
		} break;
			case 'h': case 'H':{
				cin.clear();
				cin.sync();
				cout << "Adding hue SIFT to used features" << endl;
				chosenOptions.push_back(1);
		} break;
			case 'o': case 'O':{
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(2);
		} break;
			case '1': {
				cin.clear();
				cin.sync();
				cout << "Adding normal SURF to used features" << endl;
				chosenOptions.push_back(3);
		} break;
			case '2': {
				cin.clear();
				cin.sync();
				cout << "Adding hue SURF to used features" << endl;
				chosenOptions.push_back(4);
		} break;
			case '3': {
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(5);
		} break;
			case 'r': case 'R': {
				cin.clear();
				cin.sync();
				cout << "Starting creating tree with chosen features" << endl;
				generateRandomForest(chosenOptions);
		} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}

void RFClass::rfTesting(vector<int> mode){

	if(!featureExtractor->codeBooksLoaded()){
		featureExtractor->loadCodebooks();
	}

	cout << "Starting testing the random forest algorithm on all test data" << endl;
	vector<bool> modes;//create boolean array of modes
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		modes.push_back(false);
	}
	for(unsigned int i = 0; i < mode.size(); i++){ //set the options of the algorithm
		modes[mode[i]] = true;
	}
	
	//create a string of the modeboolean
	string modestring;
	modestring.clear();
	for(int i = 0; i < featureExtractor->getAmountOfFeatures(); i++){
		if(modes[i]){
			modestring += '1';
		}
		else{
			modestring += '0';
		}
	}
	//generateRandomForest(mode);
	//read the data of the random forest
	//somehow, gets corrupted :( :( :(
	rfclassifier->read("randomforestdata" + modestring + ".yml", "randomforestdata" + modestring);
	//now we use the random forest to predict the class of each of the test images

	cv::Mat input;
	string imagePath; //path where the images are
	string filePath; //filepath for each image
	cv::Mat result;
	int** predictions = new int*[amountOfClasses];
	for(int i = 0; i < amountOfClasses; i++){
		predictions[i] = new int [testPicNum[i]+1];
	}

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_test\\"; //go to the classname folder
		cout << "procesing class: " << classNames[i] << endl;
		for(int j = 1; j <= testPicNum[i]; j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename
			cout << "processing image: " << classNames[i] << "_" << j << endl;
			input.release();
			input = cv::imread(imagePath); //Load rgb image
			result.release();
			result = featureExtractor->extractFeatures(modes,input);
			predictions[i][j] = rfclassifier->predict(result);
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

void RFClass::rfTestingmenu(){
	char option = ' ';

	vector<int> chosenOptions;
	while(option){
		cout << "For which feature type do you want to load the random forest for testing? (multiple possible)" << endl << endl
			 << "Chosen features: ";
		for(unsigned int i = 0; i < chosenOptions.size(); i++){
			cout << featureExtractor->getFeatureName(chosenOptions[i]) << " ";
		}
		cout << endl
		     << "  (S) Standard SIFT " << endl
			 << "  (H) Hue SIFT " <<  endl
			 << "  (O) Opponent SIFT "<<  endl
			 << "  (1) Normal SURF "  << endl
			 << "  (2) Hue SURF " <<  endl
			 << "  (3) Opponent SURF " <<  endl
			 << "  (R) andom forest testing" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return;
		} break;
			case 's':  case 'S':{
				cin.clear();
				cin.sync();
				cout << "Adding standard SIFT to used features" << endl;
				chosenOptions.push_back(0);
		} break;
			case 'h': case 'H':{
				cin.clear();
				cin.sync();
				cout << "Adding hue SIFT to used features" << endl;
				chosenOptions.push_back(1);
		} break;
			case 'o': case 'O':{
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(2);
		} break;
			case '1': {
				cin.clear();
				cin.sync();
				cout << "Adding normal SURF to used features" << endl;
				chosenOptions.push_back(3);
		} break;
			case '2': {
				cin.clear();
				cin.sync();
				cout << "Adding hue SURF to used features" << endl;
				chosenOptions.push_back(4);
		} break;
			case '3': {
				cin.clear();
				cin.sync();
				cout << "Adding opponent SIFT to used features" << endl;
				chosenOptions.push_back(5);
		} break;
			case 'r': case 'R': {
				cin.clear();
				cin.sync();
				cout << "Starting testing with chosen random forest options" << endl;
				rfTesting(chosenOptions);
		} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}

void RFClass::menu(void){
	char option = ' ';
	while(option){
		cout << "Choose an option: " << endl
			 << "  (C) odebook creation" << endl
			 << "  (T) raining the model" << endl
			 << "  (R) andom Forest training" << endl
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
			case 'c':  case 'C':{
				cin.clear();
				cin.sync();
				cout << "Loading codebook creation menu" << endl;
				createCodebookMenu();
		} break;
			case 't': case 'T':{
				cin.clear();
				cin.sync();
				cout << "Loading training menu" << endl;
				trainModelMenu();
		} break;
			 case 'r': case 'R':{
				cin.clear();
				cin.sync();
				cout << "Loading random forest training menu" << endl;
				rfTrainigmenu();
		} break;
			case 'a': case 'A':{
				cin.clear();
				cin.sync();
				cout << "Evaluating the test set" << endl;
				rfTestingmenu();
		} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}