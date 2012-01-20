// RGBDRecog.cpp : Defines the entry point for the console application.
//

#include <stdlib.h>
#include <crtdbg.h>

//Include files

//used for compilation
#include "stdafx.h" 


//Include other libraries
#include "FeatureVector.h" //feature vector contains SIFT features
#include <string>

#include "Winbase.h"
#include "RFClass.h"

using namespace std;
//using namespace cv; //opencv namespace

int _tmain(int argc, _TCHAR* argv[])
{
	//int mode = NORMAL; //specify the color mode used, either NORMAL, HUE or OPPONENT

	char option = ' ';

	while(option != 'q' && option != 'Q'){
		cout << "Choose a classification method: " << endl
			 << "  (R) andom forests" << endl
			 << "  (N) earest neighbor" << endl
			 << "  (Q) uit" << endl;
		option = cin.get();
		switch(option){
			case 'r': case 'R':{
				cin.clear();
				cin.sync();
				cout << "Running the random forest menu" << endl;
				RFClass* rfclass = new RFClass();
				if(rfclass!= NULL){
					rfclass->menu();
				}
				else{
					cout << "Could not create the random forest." << endl;
				}
			} break;

			case 'n': case 'N':{
				cin.clear();
				cin.sync();
				cout << "Neirest neigbor is not implemented yet" << endl;
			} break;
			
			case 'q': case 'Q':{
				cin.clear();
				cin.sync();
				cout << "exiting..." << endl;
			} break;
			default:{
				cin.clear();
				cin.sync();
				cout << "No such option exists" << endl;
			} break;
		}
	}
	return 0;
}