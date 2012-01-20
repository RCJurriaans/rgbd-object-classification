#pragma once

struct SetElement
{
	SetElement * next;
	int number;
};

struct Set
{
	Set * nextSet;
	SetElement * Element;
};

class DisjointSet
{
public:
	DisjointSet(void);
	~DisjointSet(void);

	void AddSet();
	void AddElement (int setNumber, int addThisNumber);
	int GetSmallestElementInSet(int setNumber);
	void Equalize();

	int numberOfSets;

private:
	Set * currentSet;
	SetElement * currentElement;
	Set * entrance;


	//used by destructor
	void DestroySet(Set* thisSet);
	void DestroyElement(SetElement* thisElement);
	int *smallest;
	Set ** locationPointer;
};

