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
#include "NNMenu.h"

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
			 << "  (S) VM classification" << endl
			 << "  (N) earest neighbor" << endl
			 << "  (D) ata segmentation" << endl
			 << "  (O) ptions" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'r': case 'R':{
				cin.clear();
				cin.sync();
				cout << "Running the random forest menu" << endl;
				RFClass* rfclass = new RFClass(settings,0);
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
			case 's': case 'S':{
				cin.clear();
				cin.sync();
				cout << "Running the SVM menu" << endl;
				RFClass* rfclass = new RFClass(settings,1);
				if(rfclass!= NULL){
					rfclass->menu();
				}
				else{
					cout << "Could not create the SVM menu." << endl;
				}
				if(rfclass!= NULL){
					delete rfclass;
				}
			} break;

			case 'n': case 'N':{
				cin.clear();
				cin.sync();
				//cout << "Not implemented; use RF menu" <<endl;
				cout << "Running the nearest neighbour menu" << endl;
				boost::shared_ptr<NNMenu> nnmenu(new NNMenu(settings));
				nnmenu->menu();
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
			case 'o': case 'O':{
				cin.clear();
				cin.sync();
				cout << "Running the options menu" << endl;
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