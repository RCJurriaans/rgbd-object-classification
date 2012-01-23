#include "StdAfx.h"
#include "Settings.h"


Settings::Settings(void)
{
	featureData = new FeatureData;
	settingsfile = "settings.txt";

	for(int i = 0; i < featureData->amountOfFeatures; i++){
		modes.push_back(false);
	}

	if(!readSettings()){
		cout << "defaulting to standard settings" << endl;
		segmentation = true;
		modes[0] = true;
	}
}

Settings::~Settings(void)
{
	delete featureData;
}

bool Settings::readSettings(){
	ifstream in;

	in.open(settingsfile);
	if(!in.good()){
		cout << "ERROR: Could not find " << settingsfile << endl;
		return false;
	}

	in >> segmentation;
	bool temp;
	for(unsigned int i = 0; i < modes.size(); i++){
		in >> temp;
		modes[i] = temp;
	}
	return true;
}

bool Settings::writeSettings(){
	ofstream out;

	out.open(settingsfile);
	if(!out.good()){
		cout << "ERROR: Could not find " << settingsfile << endl;
		return false;
	}

	out << segmentation;
	for(unsigned int i = 0; i < modes.size(); i++){
		out << modes[i];
	}
	return true;
}

string Settings::boolstring(bool in){
	if(in){
		return "true";
	}else{
		return "false";
	}
}

void Settings::outputSettings(){
	cout << "  Using segmentation: " << boolstring(segmentation) << endl;
	for(int i = 0; i < featureData->amountOfFeatures; i++){
		cout << "  Using option " << featureData->featureNames[i] << ": " << boolstring(modes[i]) << endl;
	}
}

string Settings::settingsString(){
	string output = "_";

	if(segmentation){
		output += '1';
	}else{
		output += '0';
	}
	for(int i = 0; i < featureData->amountOfFeatures; i++){
		if(modes[i]){
			output += '1';
		}else{
			output += '0';
		}
	}
	return output;
}

void Settings::segmentationSettings(){
	cout << "  Using segmentation: " << boolstring(segmentation) << endl;
	char option = ' ';
	bool choice = false;
	if(segmentation){
		cout << "turn segmentation off? (y/n)" << endl;
		cin.clear();cin.sync();
		option = cin.get();
		if(option == 'y' || option == 'Y'){
			choice = false;
		}else{
			choice = true;
		}
		cin.clear();cin.sync();
	}else{
		cout << "turn segmentation on? (y/n)" << endl;
		cin.clear();cin.sync();
		option = cin.get();
		if(option == 'y' || option == 'Y'){
			choice = true;
		}else{
			choice = false;
		}
		cin.clear();cin.sync();
	}
	segmentation = choice;
}

void Settings::featureSettings(){
	cout << "Current settings:" << endl;
	for(int i = 0; i < featureData->amountOfFeatures; i++){
		cout << "  Option " << i  <<" " << featureData->featureNames[i] << ": " << boolstring(modes[i]) << endl;
	}
	cout << "Which settings do you want to change? (give the number)" << endl;
	int option;
	option = cin.get();

	if(option < 0 || option > featureData->amountOfFeatures){
		cout << "invalid feature";
		return;
	}

	if(modes[option]){
		modes[option] = false;
	}else{
		modes[option] = true;
	}
}

void Settings::menu(){
	cout << "Current settings:" << endl;
	outputSettings();


	char option = ' ';
		while(option != 'q' && option != 'Q'){
		cout << "Which settings do you want to change?" << endl
			 << "  (S) egmentation" << endl
			 << "  (F) eatures" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 's': case 'S':{
				cin.clear();
				cin.sync();
				segmentationSettings();
			} break;
			case 'f': case 'F':{
				cin.clear();
				cin.sync();
				featureSettings();
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