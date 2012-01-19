#include "StdAfx.h"
#include "DisjointSet.h"
#include <iostream>
using namespace std;

DisjointSet::DisjointSet(void)
{
	entrance = new Set;
	entrance->nextSet = NULL;
	numberOfSets = 1;
	entrance->Element = new SetElement;
	entrance->Element->next = NULL;
	entrance->Element->number = numberOfSets;
}


DisjointSet::~DisjointSet(void)
{
	DestroySet(entrance);
}

void DisjointSet::DestroySet(Set* thisSet)
{
	if(smallest){
		//delete [] smallest;
	}
	if(locationPointer){
		//delete [] locationPointer;
	}
	Set* destroyThisSet = thisSet->nextSet;
	SetElement* destroyThisElement = thisSet->Element;
	delete thisSet;
	if(destroyThisSet != NULL){
		DestroySet(destroyThisSet);
	}
	if(destroyThisElement != NULL){
		DestroyElement(destroyThisElement);
	}
}

void DisjointSet::DestroyElement(SetElement* thisElement)
{
	SetElement* destroyThisElement = thisElement->next;
	delete thisElement;
	if(destroyThisElement != NULL){
		DestroyElement(destroyThisElement);
	}
}


void DisjointSet:: AddSet()
{
	numberOfSets++;
	currentSet = entrance;
	while(currentSet->nextSet != NULL)
	{
		currentSet = currentSet->nextSet;
	} 
	//now were pointing to the last set
	currentSet->nextSet = new Set;
	currentSet = currentSet->nextSet;
	currentSet->nextSet = NULL;
	currentSet->Element = new SetElement;
	currentSet->Element->number = numberOfSets;
	currentSet->Element->next = NULL;
}

void DisjointSet:: AddElement (int setNumber, int addThisNumber)
{
	if(setNumber < addThisNumber){ //switch biggest to setNumber
		int temp = addThisNumber;
		addThisNumber = setNumber;
		setNumber = temp;
	}
	
	if(addThisNumber <1){
		return;
	}

	currentSet = entrance;
	int counter = 1;
	if(setNumber >numberOfSets){
		return; //<TODO> ERROR MESSAGE no such setnumber
	}
	while(currentSet->nextSet != NULL && counter < setNumber){ //there is always one set, created upon init
		currentSet = currentSet->nextSet;
		counter++;
	} //currentSet is now on wanted set!
	currentElement = currentSet->Element; //there is always one element created upon set init
	while(currentElement->number != addThisNumber && currentElement->next != NULL){
		currentElement = currentElement->next;
	}//currentElement is now on last spot, and no equivalent number has been found
	if(currentElement->number != addThisNumber){
		currentElement->next = new SetElement;
		currentElement = currentElement->next;
		currentElement->number = addThisNumber;
		currentElement->next = NULL;
	}
}

int DisjointSet:: GetSmallestElementInSet(int setNumber)
{

	return smallest[setNumber-1];
	//currentSet = entrance;
	//int counter = 1;
	//int smallest = 10000000;
	//if(setNumber >numberOfSets){
	//	return 0; //<TODO> ERROR MESSAGE no such setnumber
	//}
	//while(currentSet->nextSet != NULL && counter < setNumber){
	//	currentSet = currentSet->nextSet;
	//	counter++;
	//} //currentSet is now on wanted set!
	//currentElement = currentSet->Element;
	//while(currentElement->next != NULL){
	//	if(currentElement->number < smallest){
	//		smallest = currentElement->number;
	//	}
	//	currentElement = currentElement->next;
	//}//currentElement is now on last spot, and no equivalent number has been found
	//if(currentElement->number < smallest){
	//		smallest = currentElement->number;
	//}
	//return smallest;
}


//makes sure that for element x in set y, element y is also in set x
void DisjointSet::Equalize(){ //<TODO> Only add extra element when set_counter < number
	smallest = new int[numberOfSets];
	locationPointer = new Set*[numberOfSets];

	currentSet = entrance;
	int currentSmallest = 0;
	bool _break = false;

	for(int i = 0; i < numberOfSets; i++){
		locationPointer[i] = currentSet;
		currentSet = currentSet->nextSet;
	} //now we have an array of pointers that point towards each set
	//For each number, trace back the smallest set it belongs to
	for(int i = 0; i < numberOfSets; i++){//for each set
		currentSmallest = i+1; //set always has its own number in... actually -1 this
		// we find a smaller number in the current set
		_break = false;
		do{	
			currentSet = locationPointer[currentSmallest-1]; //we start looking in this set
			currentElement = currentSet->Element;
			while(currentElement != NULL && currentElement->number >= currentSmallest){ //
					currentElement = currentElement->next;
				}
			//now if we are not done, currentElement->number has a smaller number
			if(currentElement != NULL){
				currentSmallest = currentElement->number;
			}else{_break = true;}
		}while(!_break);
		smallest[i] = currentSmallest;
	}
}