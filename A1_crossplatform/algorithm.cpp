#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <unordered_map>
#include <string>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <functional>

#include <iomanip>
#include <fstream>


#include "algorithm.h"

using namespace std;

const int PUZZLE_SIZE = 3;
const int NUM_MOVES = 4;

const vector<int> DX = {0, 1, 0, -1};
const vector<int> DY = {1, 0, -1, 0};

struct PuzzleState {
    string state;
    string path;
    int g; // path cost (number of moves)
    int h; // heuristic value

    PuzzleState(string s, string p, int g_cost, int h_cost) : state(s), path(p), g(g_cost), h(h_cost) {}

    // Priority is based on f(n) = g(n) + h(n)
    bool operator<(const PuzzleState &other) const {
        return (g + h) > (other.g + other.h); // min-heap
    }
};

bool isValid(int x, int y) {
    return (x >= 0 && x < PUZZLE_SIZE && y >= 0 && y < PUZZLE_SIZE);
}

vector<pair<string, char>> get_successors(const string &state) {
    vector<pair<string, char>> successors;
    int zero_index = state.find('0');

    // Define possible moves and their directions (Up, Right, Down, Left)
    static const vector<pair<int, char>> directions = {
        {-3, 'U'}, {1, 'R'}, {3, 'D'}, {-1, 'L'}
    };

    // Check for valid moves based on the blank tile's position
    for (const auto& [move, dir] : directions) {
        int new_index = zero_index + move;
        if (new_index >= 0 && new_index < 9) {
            if ((zero_index % 3 == 0 && dir == 'L') || (zero_index % 3 == 2 && dir == 'R')) {
                continue; // Invalid left or right move due to edge
            }
            string new_state = state;
            swap(new_state[zero_index], new_state[new_index]);
            successors.push_back({new_state, dir});
        }
    }

    return successors;
}

// Heuristic function for Misplaced Tiles
int misplacedTilesF(const string& state, const string& goalState) {
    int misplaced = 0;
    for (int i = 0; i < 9; ++i) {
        if (state[i] != '0' && state[i] != goalState[i]) {
            misplaced++;
        }
    }
    return misplaced;
}

// Heuristic function for Manhattan Distance
int manhattanDistanceF(const string& state, const string& goalState) {
    int distance = 0;
    for (int i = 0; i < 9; ++i) {
        if (state[i] != '0') {
            int goalIndex = goalState.find(state[i]);
            distance += abs(i / 3 - goalIndex / 3) + abs(i % 3 - goalIndex % 3);
        }
    }
    return distance;
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
    
    clock_t startTime = clock();;
    priority_queue<PuzzleState> pq;
    set<string> visited;

    pq.push(PuzzleState(initialState, "", 0, 0));
    visited.insert(initialState);
    maxQLength = 1;
    numOfStateExpansions = 0;
    numOfDeletionsFromMiddleOfHeap=0;
    numOfLocalLoopsAvoided=0;
    numOfAttemptedNodeReExpansions=0;
	actualRunningTime=0.0;	

    string path;

    while (!pq.empty()) {
        //maxQLength = max(maxQLength, (int)openQueue.size());

        PuzzleState current = pq.top();
        pq.pop();

        if (current.state == goalState) {
            pathLength = current.g;
            actualRunningTime = float(clock() - startTime) / CLOCKS_PER_SEC;
            return current.path;
        }

        /*
        if (closedSet.find(current.state) != closedSet.end()) {     //redundent
            continue;
        }

        closedSet.insert(current.state);
        */
        
        ++numOfStateExpansions;
        vector<pair<string, char>> successors = get_successors(current.state);
        
        for (const auto& [successor, direction] : successors) {
            if (visited.find(successor) == visited.end()) {
                visited.insert(successor);
                pq.push(PuzzleState(successor, current.path + direction, current.g + 1, 0));
            }
        }

        maxQLength = max(maxQLength, (int)pq.size());
    }
	
//***********************************************************************************************************
	actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC); 
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
    clock_t startTime = clock();;
   
    // Priority queue (min-heap) for open list
    vector<PuzzleState> openList;
    make_heap(openList.begin(), openList.end());
    set<string> closedList;

    pathLength = 0;
    numOfStateExpansions = 0;
    maxQLength = 0;

    //calc start node h value
    int startHeuristic;
    //if misplasedTiles
    if (heuristic == 0){
        startHeuristic = misplacedTilesF(initialState, goalState);
    }
    //if manhattanDistance
    else if (heuristic==1){
        startHeuristic = manhattanDistanceF(initialState, goalState);
    }


    // Push initial state into the heap with f(n) = g(n) + h(n)
    PuzzleState startNode = PuzzleState(initialState, "", 0, startHeuristic);
    openList.push_back(startNode);
    push_heap(openList.begin(), openList.end());


    // A* Search Loop
    while (!openList.empty()) {
        pop_heap(openList.begin(), openList.end());
        PuzzleState currentNode = openList.back();
        openList.pop_back();

        // Check if goal state is reached
        if (currentNode.state == goalState) {
            pathLength = currentNode.g;
            actualRunningTime = (float)(clock() - startTime) / CLOCKS_PER_SEC;
            return currentNode.path; // Return the path leading to the goal
        }

        // Add to closed list
        closedList.insert(currentNode.state);
        numOfStateExpansions++;

        // Generate successors
        vector<pair<string, char>> successors = get_successors(currentNode.state);

        // Expand each successor
        for (const auto& [newState, move] : successors) {
            if (closedList.find(newState) != closedList.end()) {
                continue; // Skip if already expanded
            }

            // Compute g and h for the successor
            int newG = currentNode.g + 1;

            //check which heuristic to use
            int newH;
            //if misplasedTiles
            if (heuristic == 0){
                newH = misplacedTilesF(newState, goalState);
            }
            //if manhattanDistance
            else if (heuristic==1){
                newH = manhattanDistanceF(newState, goalState);
            }

            PuzzleState successorNode{newState,  currentNode.path + move, newG, newH};

            openList.push_back(successorNode);
            push_heap(openList.begin(), openList.end());

            // Update max queue length
            maxQLength = max(maxQLength, (int)openList.size());
        }
    }

	actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);                         
	
	return path;		
		
}

