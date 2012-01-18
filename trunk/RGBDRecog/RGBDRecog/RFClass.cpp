#include "StdAfx.h"
#include "RFClass.h"
#include "FeatureVector.h"
#include <fstream>
#include <time.h>
using namespace cv;
using namespace std;



RFClass::RFClass(void){
	amountOfPossibleFeatures = 6;

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
		traingPicNum.push_back(tempi);
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
		cout << "  " << classNames[i] << ' ' << traingPicNum[i] << ' ' << testPicNum[i] << endl;
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
}



RFClass::~RFClass(void){
}

//converts a number to a fixed length string, adding zeros in front
string RFClass:: convertNumberToFLString(int length, int number){

	int tempnum = number;
	length--;
	string returnvalue;
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
	const char *_COLORNAMES[] = {"NORMALSIFT", "HUESIFT", "OPPONENTSIFT","NORMALSURF", "HUESURF", "OPPONENTSURF"};
	cout << "Running codebook creation with setting: " << _COLORNAMES[mode] << endl;
	cout << "parameters are from data_info.txt:" << endl;
	cout << "  Dictionary size: " << dicsize << endl;
	cout << "  SIFT Threshold Scale: " << SIFTThreshScale << endl;
	cout << "Initializing codebook creation files..." << endl;
	cout << "Assuming that the training data files are located in: " << getenv("RGBDDATA_DIR") << endl;

	clock_t init, final; //for timing
	init=clock();

	string imagePath; //path where the images are
	string filePath; //filepath for each image
	Mat tempinput; //temp
	vector<Mat> input; //stores the image data
	vector<Mat> planes; //temp file for splitting the image data
	FeatureVector* fv = new FeatureVector; //this contains all features that are extacted
	SIFT siftdetector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE ); //sift class, ugly initialization of all parameters...
	SURF surfdetector;
    vector<KeyPoint> keypoints; //temp variable that contains the keypoints found for each img
	Mat descriptors; //temp variable used to store the extracted SIFT descriptors for every image
	SiftDescriptorExtractor* SDE = new cv::SiftDescriptorExtractor(); //Sift descriptor extraction class
	OpponentColorDescriptorExtractor des(SDE); //written by van de Sande... extracts the opponent SIFT data
	SurfDescriptorExtractor* SuDE = new cv::SurfDescriptorExtractor();
	OpponentColorDescriptorExtractor desSURF(SuDE);
	Mat grayimg; //temp used to store the grayimg in for Opponent color

	vector<float> tempfloatdesc;

	cout << "Running image feature extraction" << endl;
			cout << "Using the first half of the training images to create the codebooks" << endl;
	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << endl;
		for(int j = 1; j <= floor(static_cast<double>(traingPicNum[i])/2); j++){ //for each image
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename
			cout << "processing on image: " << classNames[i] << "_" << j << endl;
			if(mode == 0 || mode == 3){ //every step cleans up all the matrices, and adds new image data
				input.clear();
				input.push_back(cv::imread(imagePath,0)); //Load as grayscale image
			}else if (mode == 1 || mode == 4){
				input.clear();
				tempinput.release();
				planes.clear();
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input.push_back(planes[0]);
				input.push_back(planes[2]);
			}else if (mode == 2 || mode == 5){
				tempinput.release();
				tempinput = cv::imread(imagePath);
			}
			if (mode == 0||mode == 3){ //use the gotten image data to compute the features, and add them to the feature vector
				descriptors.release(); //still clear all data before reinitializing
				if(mode == 0){
					siftdetector(input[0],Mat(),vector<KeyPoint>(),descriptors);
				}else if (mode == 3){
					tempfloatdesc.clear();
					tempinput.release();
					surfdetector(input[0],Mat(),keypoints,tempfloatdesc);
					tempinput = Mat(tempfloatdesc).reshape(1,keypoints.size());
					descriptors = tempinput;
				}
				fv->AddFeatures(descriptors);
			} else if(mode == 1|| mode == 4){
				descriptors.release();
				keypoints.clear();
				if(mode == 1){
					siftdetector(input[1],Mat(),keypoints,Mat());
					siftdetector(input[0],Mat(),keypoints,descriptors,true);
				} else if (mode == 4){
					tempinput.release();
					tempfloatdesc.clear();
					surfdetector(input[1],Mat(),keypoints,vector<float>());
					surfdetector(input[1],Mat(),keypoints,tempfloatdesc,true);
					tempinput = Mat(tempfloatdesc).reshape(1,keypoints.size());
					descriptors = tempinput;
				}
				fv->AddFeatures(descriptors);
			} else if(mode == 2|| mode == 5){
				descriptors.release();
				keypoints.clear();
				grayimg.release();
				cvtColor(tempinput,grayimg,CV_RGB2GRAY);
				if(mode == 2){
					siftdetector(grayimg,Mat(),keypoints,Mat());
					des.compute(tempinput,keypoints,descriptors);
				} else if (mode == 5){
					surfdetector(grayimg,Mat(),keypoints,vector<float>());
					desSURF.compute(tempinput,keypoints,descriptors);
				}
				fv->AddFeatures(descriptors);
			}
		}
	}
	cout << "found " << fv->features->rows << " descriptors."<< endl;
	cout << "Running kmeans" << endl;
	Mat* codebook = fv->kmeans(dicsize); //run kmeans on the data for dicsize clusters
	cout << "writing the result to file: " << "codebook" + boost::lexical_cast<string>(mode) + ".yml" << endl;
	FileStorage fs("codebook" + boost::lexical_cast<string>(mode) + ".yml", FileStorage::WRITE); //store the codebook to a yaml file
	fs << "codebook" + boost::lexical_cast<string>(mode) << *codebook;
	
	//delete all data... just for security :/
	fv->~FeatureVector();
	tempinput.release();
	for(unsigned int i = 0; i < input.size(); i++)
	{
		input[i].release();
	}
	for(unsigned int i = 0; i < planes.size(); i++)
	{
		planes[i].release();
	}
	descriptors.release();
	des.empty();
	grayimg.release();
	delete fv;
	delete codebook;
	final=clock()-init;
	
	cout <<  "Done creating the " << _COLORNAMES[mode] << " codebook in "  << (double)final / ((double)CLOCKS_PER_SEC*60) << " minutes" <<endl;
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
	for(int i = 0; i < amountOfPossibleFeatures; i++){
		FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", FileStorage::READ);
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

void RFClass::addDescriptor(bool & firstadded, Mat & tempfeaturevector, FeatureVector * codebooks,Mat & descriptors, int mode){
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
	const char *_COLORNAMES[] = {"NORMALSIFT", "HUESIFT", "OPPONENTSIFT","NORMALSURF", "HUESURF", "OPPONENTSURF"};
	for(int i = 0; i < amountOfPossibleFeatures; i++){
		modes.push_back(false);
	}
	for(unsigned int i = 0; i < mode.size(); i++){ //set the options of the algorithm
		modes[mode[i]] = true;
	}
	
	cout << "Running training with settings: ";
	for(unsigned int i = 0; i < modes.size(); i++){
		if(modes[i]){
			cout << _COLORNAMES[i] << " ";
		}
	}
	cout << "parameters are from data_info.txt:" << endl;
	cout << "  Dictionary size: " << dicsize << endl;
	cout << "  SIFT Threshold Scale: " << SIFTThreshScale << endl;
	cout << "Initializing training creation files..." << endl;
	cout << "Assuming that the training data files are located in: " << getenv("RGBDDATA_DIR") << endl;

	FeatureVector *codebooks;
	codebooks = new FeatureVector[amountOfPossibleFeatures];

	//load the codebook data
	for(int i = 0; i < amountOfPossibleFeatures; i++){
		if(modes[i] == true){
			FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", FileStorage::READ);
			Mat M; fs["codebook" + boost::lexical_cast<string>(i)] >> M;
			fs.release();
			codebooks[i].AddFeatures(M);
			codebooks[i].TrainkNN();
		}
	}

	string imagePath; //path where the images are
	string filePath; //filepath for each image
	Mat tempinput; //temp
	vector<Mat> input; //stores the image data
	vector<Mat> planes; //temp file for splitting the image data
	SIFT siftdetector(SIFTThreshScale*(0.04 / SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS), 10,
          SIFT::CommonParams::DEFAULT_NOCTAVES,
          SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
          SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
          SIFT::CommonParams::FIRST_ANGLE ); //sift class, ugly initialization of all parameters...
	SURF surfdetector;
    vector<KeyPoint> keypoints; //temp variable that contains the keypoints found for each img
	Mat descriptors; //temp variable used to store the extracted SIFT descriptors for every image
	SiftDescriptorExtractor * SDE = new cv::SiftDescriptorExtractor(); //Sift descriptor extraction class
	OpponentColorDescriptorExtractor des(SDE); //written by van de Sande... extracts the opponent SIFT data
	SurfDescriptorExtractor* SuDE = new cv::SurfDescriptorExtractor();
	OpponentColorDescriptorExtractor desSURF(SuDE);
	Mat grayimg; //temp used to store the grayimg in for Opponent color 

	vector<float> tempfloatdesc;
	Mat tempfeaturevector;
	vector<Mat> featurevector;

	for(int i = 0; i < amountOfClasses; i++){
		filePath = getenv("RGBDDATA_DIR"); //get the proper environment variable path for the data
		filePath += "\\" + classNames[i] + "_train\\"; //go to the classname folder
		cout << "starting processing class: " << classNames[i] << endl;
		for(int j = static_cast<int>(floor(static_cast<double>(traingPicNum[i])/2))+1; j <= traingPicNum[i]; j++){ //for each image
			bool firstadded = false;
			imagePath = filePath + "img" + convertNumberToFLString(3,j) + fileExtension; //get the proper filename
			cout << "processing on image: " << classNames[i] << "_" << j << endl;
			if(modes[0] || modes[3]){ //every step cleans up all the matrices, and adds new image data
				input.clear();
				input.push_back(cv::imread(imagePath,0)); //Load as grayscale image
			}else if (modes[1] || modes[4]){
				input.clear();
				tempinput.release();
				tempinput = cv::imread(imagePath);
				cvtColor(tempinput,tempinput,CV_BGR2HSV);
				split(tempinput,planes);
				input.push_back(planes[0]);
				input.push_back(planes[2]);
			}else if (modes[2] || modes[5]){
				tempinput.release();
				tempinput = cv::imread(imagePath);
			}
			if (modes[0] || modes[3]){ //use the gotten image data to compute the features, and add them to the feature vector
				descriptors.release(); //still clear all data before reinitializing
				if(modes[0]){
					siftdetector(input[0],Mat(),vector<KeyPoint>(),descriptors);	
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,0);
				}
				if(modes[3]){
					tempfloatdesc.clear();
					tempinput.release();
					surfdetector(input[0],Mat(),keypoints,tempfloatdesc);
					tempinput = Mat(tempfloatdesc).reshape(1,keypoints.size());
					descriptors = tempinput;
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,3);
				}
			} else if(modes[1] || modes[4]){
				descriptors.release();
				keypoints.clear();
				if(modes[1]){
					siftdetector(input[1],Mat(),keypoints,Mat());
					siftdetector(input[0],Mat(),keypoints,descriptors,true);
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,1);
				}
				if(modes[4]){
					tempinput.release();
					tempfloatdesc.clear();
					surfdetector(input[1],Mat(),keypoints,vector<float>());
					surfdetector(input[1],Mat(),keypoints,tempfloatdesc,true);
					tempinput = Mat(tempfloatdesc).reshape(1,keypoints.size());
					descriptors = tempinput;
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,5);
				}
			} else if(modes[2] || modes[5]){
				descriptors.release();
				keypoints.clear();
				grayimg.release();
				cvtColor(tempinput,grayimg,CV_RGB2GRAY);
				if(modes[2]){
					siftdetector(grayimg,Mat(),keypoints,Mat());
					des.compute(tempinput,keypoints,descriptors);
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,2);
				}
				if(modes[5]){
					surfdetector(grayimg,Mat(),keypoints,vector<float>());
					desSURF.compute(tempinput,keypoints,descriptors);
					addDescriptor(firstadded,tempfeaturevector,codebooks,descriptors,5);
				}
			}
			
			//tempfeaturevector now contains all found feature descriptors,
			//we can add this to this class
			if(j == floor(static_cast<double>(traingPicNum[i])/2)+1){
				featurevector.push_back(tempfeaturevector);
			}else{
				hconcat(featurevector[i],tempfeaturevector,featurevector[i]);
			}
		}
	}
	string modestring;
	modestring.clear();
	for(int i = 0; i < amountOfPossibleFeatures; i++){
		if(modes[i]){
			modestring += '1';
		}
		else{
			modestring += '0';
		}
	}

	cout << "Done creating the feature vectors for the training set" << endl
		 << "Saving the data to " << "trainingdata" + modestring + "RF" + ".yml";



	FileStorage fs2("trainingdata" + modestring + "RF" ".yml", FileStorage::WRITE); //store the codebook to a yaml file
	for(int i = 0; i < amountOfClasses;i++){
		fs2 << "trainingdata" + modestring + boost::lexical_cast<string>(i) << featurevector[i];
	}
	fs2.release();

	cout << "Saved trainingdata" << endl;
	cout << "Starting training of random forest" << endl;

	Mat mergedData;
	mergedData = featurevector[0];
	for(int i = 1; i < amountOfClasses; i++){
		hconcat(mergedData,featurevector[i],mergedData);
	}
	CvRTrees* treestructure = new CvRTrees();
	Mat responses(mergedData.cols,1,mergedData.type());
	for(int i = 0; i < amountOfClasses;i++){
		int multifactor=0;
		for(int j = 0; j < i; j++){
			multifactor+= traingPicNum[j]-static_cast<int>(floor(static_cast<double>(traingPicNum[j])/2));
		}
		for(int j = 0; j < featurevector[i].cols; j++){
			responses.at<float>(j+multifactor) = static_cast<float>(i);
		}
	}
	treestructure->train(mergedData,CV_COL_SAMPLE,responses,Mat(),Mat(),Mat(),Mat(),CvRTParams());

	cout << "Random forest trained, saving the data to " << "randomforestdata" + modestring + ".yml" << endl;
	string filename = "randomforestdata" + modestring;
	string dataname = "randomforestdata" + modestring + ".yml";
	treestructure->write(cvOpenFileStorage(dataname.c_str(), 0, CV_STORAGE_WRITE_TEXT ), filename.c_str());
	cout << "Done saving the random forest data." << endl;
	delete[]codebooks;
}

void RFClass::trainModelMenu(){
		char option = ' ';
	vector<bool> foundCodebooks;
	for(int i = 0; i < amountOfPossibleFeatures; i++){
		FileStorage fs("codebook" + boost::lexical_cast<string>(i) + ".yml", FileStorage::READ);
		if(fs.isOpened()){
			foundCodebooks.push_back(true);
		}
		else{
			foundCodebooks.push_back(false);
		}
	}

	vector<int> chosenOptions;
	const char *_COLORNAMES[] = {"NORMALSIFT", "HUESIFT", "OPPONENTSIFT","NORMALSURF", "HUESURF", "OPPONENTSURF"};
	while(option){
		cout << "For which feature type do you want to train the data? (multiple possible)" << endl << endl
			 << "Chosen features: ";
		for(unsigned int i = 0; i < chosenOptions.size(); i++){
			cout << _COLORNAMES[chosenOptions[i]] << " ";
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

void RFClass::menu(void){
	char option = ' ';
	while(option){
		cout << "Choose an option: " << endl
			 << "  (C) odebook creation" << endl
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
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			}break;
		}
	}
}