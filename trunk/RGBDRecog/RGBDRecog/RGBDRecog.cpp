// RGBDRecog.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <crtdbg.h>

//Include files

//used for compilation
#include "stdafx.h" 


//Include other libraries
//#include "FeatureVector.h" //feature vector contains SIFT features
#include <string>

//#include "Winbase.h"
#include "RFClass.h"
#include "DataSegmenter.h"
#include "Settings.h"

using namespace std;
//using namespace cv; //opencv namespace

int _tmain(int argc, _TCHAR* argv[])
{
	//int mode = NORMAL; //specify the color mode used, either NORMAL, HUE or OPPONENT

	char option = ' ';

	Settings * settings = new Settings();

	while(option != 'q' && option != 'Q'){
		cout << "Choose a classification method: " << endl
			 << "  (R) andom forests" << endl
			 << "  (N) earest neighbor" << endl
			 << "  (D) ata segmentation" << endl
			 << "  (S) ettings" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'r': case 'R':{
				cin.clear();
				cin.sync();
				cout << "Running the random forest menu" << endl;
				RFClass* rfclass = new RFClass(settings);
				if(rfclass!= NULL){
					rfclass->menu();
				}
				else{
					cout << "Could not create the random forest." << endl;
				}
				if(rfclass!= NULL){
					delete rfclass;
				}
			} break;

			case 'n': case 'N':{
				cin.clear();
				cin.sync();
				cout << "Running the nearest neighbour menu" << endl;
				boost::shared_ptr<NNMenu> nnmenu(new NNMenu());
				nnmenu->showMenu();
			} break;
			case 'd': case 'D':{
				cin.clear();
				cin.sync();
				cout << "Running the data segmentation menu"<< endl;
				DataSegmenter * ds = new DataSegmenter();
				if(ds!= NULL){
					ds->menu();
				}
				else{
					cout << "Could not create the random forest." << endl;
				}
				if(ds!= NULL){
					delete ds;
				}
			} break;
			case 's': case 'S':{
				cin.clear();
				cin.sync();
				cout << "Running the settings menu" << endl;
				settings->menu();
			} break;
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
				return 0;
			} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			} break;
		}
	}

	delete settings;
	return 0;
}