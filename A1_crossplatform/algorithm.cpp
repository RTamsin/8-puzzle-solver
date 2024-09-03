
#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <functional>
#include <algorithm>
#include "algorithm.h"

using namespace std;

const int PUZZLE_SIZE = 3;
const int NUM_MOVES = 4;

const vector<int> DX = {-1, 0, 1, 0};
const vector<int> DY = {0, 1, 0, -1};

struct Node {
    string state;
    string path;
    int cost;
    int heuristic;

    Node(const string &s, const string &p, int c, int h)
        : state(s), path(p), cost(c), heuristic(h) {}

    bool operator>(const Node &other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

bool isValid(int x, int y) {
    return (x >= 0 && x < PUZZLE_SIZE && y >= 0 && y < PUZZLE_SIZE);
}

vector<Node> generateSuccessors(const Node &node) {
    vector<Node> successors;
    int pos = node.state.find('0');
    int x = pos / PUZZLE_SIZE;
    int y = pos % PUZZLE_SIZE;

    for (int i = 0; i < NUM_MOVES; ++i) {
        int newX = x + DX[i];
        int newY = y + DY[i];
        if (isValid(newX, newY)) {
            int newPos = newX * PUZZLE_SIZE + newY;
            string newState = node.state;
            swap(newState[pos], newState[newPos]);
            string newPath = node.path + (i == 0 ? 'U' : (i == 1 ? 'R' : (i == 2 ? 'D' : 'L')));
            successors.emplace_back(newState, newPath, node.cost + 1, 0);
        }
    }
    return successors;
}


///////////////////////////////////////////////////////////////////////////////////////////
//
// Search Algorithm:  UC with Strict Expanded List
//
// Move Generator:  
//
////////////////////////////////////////////////////////////////////////////////////////////
string uc_explist(string const initialState, string const goalState, int& pathLength, int &numOfStateExpansions, int& maxQLength,
                               float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap, int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions){
											 
   string path;
   clock_t startTime;
   
   numOfDeletionsFromMiddleOfHeap=0;
   numOfLocalLoopsAvoided=0;
   numOfAttemptedNodeReExpansions=0;


    // cout << "------------------------------" << endl;
    // cout << "<<uc_explist>>" << endl;
    // cout << "------------------------------" << endl;
	actualRunningTime=0.0;	
	startTime = clock();

	/* these are for test to see if code works together
    srand(time(NULL)); //RANDOM NUMBER GENERATOR - ONLY FOR THIS DEMO.  YOU REALLY DON'T NEED THIS! DISABLE THIS STATEMENT.
	maxQLength= rand() % 200; //AT THE MOMENT, THIS IS JUST GENERATING SOME DUMMY VALUE.  YOUR ALGORITHM IMPLEMENTATION SHOULD COMPUTE THIS PROPERLY.
	numOfStateExpansions = rand() % 200; //AT THE MOMENT, THIS IS JUST GENERATING SOME DUMMY VALUE.  YOUR ALGORITHM IMPLEMENTATION SHOULD COMPUTE THIS PROPERLY
    */

    priority_queue<Node, vector<Node>, greater<Node>> openQueue;
    set<string> closedSet;
    int numDeletions = 0;

    openQueue.emplace(initialState, "", 0, 0);
    maxQLength = 0;
    numOfStateExpansions = 0;

    while (!openQueue.empty()) {
        maxQLength = max(maxQLength, (int)openQueue.size());

        Node current = openQueue.top();
        openQueue.pop();

        if (current.state == goalState) {
            pathLength = current.path.length();
            actualRunningTime = float(clock() - startTime) / CLOCKS_PER_SEC;
            return current.path;
        }

        if (closedSet.find(current.state) != closedSet.end()) {
            continue;
        }
        closedSet.insert(current.state);
        ++numOfStateExpansions;

        vector<Node> successors = generateSuccessors(current);
        for (auto &successor : successors) {
            if (closedSet.find(successor.state) == closedSet.end()) {
                openQueue.push(successor);
            } else {
                ++numDeletions;
            }
        }
    }
	
//***********************************************************************************************************
	actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);
	path = "DDRRLLLUUURDLUDURDLUU"; //this is just a dummy path for testing the function
	pathLength = path.size();
	return path;		
		
}




///////////////////////////////////////////////////////////////////////////////////////////
//
// Search Algorithm:  A* with the Strict Expanded List
//
// Move Generator:  
//
////////////////////////////////////////////////////////////////////////////////////////////
string aStar_ExpandedList(string const initialState, string const goalState, int& pathLength, int &numOfStateExpansions, int& maxQLength,
                               float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap, int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions, heuristicFunction heuristic){
											 
   string path;
   clock_t startTime;
   
   numOfDeletionsFromMiddleOfHeap=0;
   numOfLocalLoopsAvoided=0;
   numOfAttemptedNodeReExpansions=0;


    // cout << "------------------------------" << endl;
    // cout << "<<aStar_ExpandedList>>" << endl;
    // cout << "------------------------------" << endl;
	actualRunningTime=0.0;	
	startTime = clock();
	srand(time(NULL)); //RANDOM NUMBER GENERATOR - ONLY FOR THIS DEMO.  YOU REALLY DON'T NEED THIS! DISABLE THIS STATEMENT.
	maxQLength= rand() % 200; //AT THE MOMENT, THIS IS JUST GENERATING SOME DUMMY VALUE.  YOUR ALGORITHM IMPLEMENTATION SHOULD COMPUTE THIS PROPERLY.
	numOfStateExpansions = rand() % 200; //AT THE MOMENT, THIS IS JUST GENERATING SOME DUMMY VALUE.  YOUR ALGORITHM IMPLEMENTATION SHOULD COMPUTE THIS PROPERLY


	
	
//***********************************************************************************************************
	actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);                         //this seems to always return null look at it later
	path = "DDRRLLLUUURDLUDURDLUU"; //this is just a dummy path for testing the function
	pathLength = path.size();
	return path;		
		
}

