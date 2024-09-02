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

const int PUZZLE_SIZE = 3;
const int NUM_MOVES = 4;

const std::vector<int> DX = {-1, 0, 1, 0};
const std::vector<int> DY = {0, 1, 0, -1};

bool isValid(int x, int y) {
    return (x >= 0 && x < PUZZLE_SIZE && y >= 0 && y < PUZZLE_SIZE);
}

int manhattanDistance(const std::string &state, const std::string &goal) {
    int distance = 0;
    for (int i = 0; i < PUZZLE_SIZE * PUZZLE_SIZE; ++i) {
        if (state[i] != '0') {
            int currentX = i / PUZZLE_SIZE;
            int currentY = i % PUZZLE_SIZE;
            int goalX = goal.find(state[i]) / PUZZLE_SIZE;
            int goalY = goal.find(state[i]) % PUZZLE_SIZE;
            distance += abs(currentX - goalX) + abs(currentY - goalY);
        }
    }
    return distance;
}

int misplacedTiles(const std::string &state, const std::string &goal) {
    int count = 0;
    for (int i = 0; i < PUZZLE_SIZE * PUZZLE_SIZE; ++i) {
        if (state[i] != '0' && state[i] != goal[i]) {
            ++count;
        }
    }
    return count;
}

struct Node {
    std::string state;
    std::string path;
    int cost;
    int heuristic;

    Node(const std::string &s, const std::string &p, int c, int h)
        : state(s), path(p), cost(c), heuristic(h) {}

    bool operator>(const Node &other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

std::vector<Node> generateSuccessors(const Node &node) {
    std::vector<Node> successors;
    int pos = node.state.find('0');
    int x = pos / PUZZLE_SIZE;
    int y = pos % PUZZLE_SIZE;

    for (int i = 0; i < NUM_MOVES; ++i) {
        int newX = x + DX[i];
        int newY = y + DY[i];
        if (isValid(newX, newY)) {
            int newPos = newX * PUZZLE_SIZE + newY;
            std::string newState = node.state;
            std::swap(newState[pos], newState[newPos]);
            std::string newPath = node.path + (i == 0 ? 'U' : (i == 1 ? 'R' : (i == 2 ? 'D' : 'L')));
            successors.emplace_back(newState, newPath, node.cost + 1, 0);
        }
    }
    return successors;
}

std::string uniformCostSearch(const std::string &initialState, const std::string &goalState,
                             int &pathLength, int &numOfStateExpansions, int &maxQLength, float &actualRunningTime) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openQueue;
    std::set<std::string> closedSet;
    int numDeletions = 0;

    clock_t start = clock();

    openQueue.emplace(initialState, "", 0, 0);
    maxQLength = 0;
    numOfStateExpansions = 0;

    while (!openQueue.empty()) {
        maxQLength = std::max(maxQLength, (int)openQueue.size());

        Node current = openQueue.top();
        openQueue.pop();

        if (current.state == goalState) {
            pathLength = current.path.length();
            actualRunningTime = float(clock() - start) / CLOCKS_PER_SEC;
            return current.path;
        }

        if (closedSet.find(current.state) != closedSet.end()) {
            continue;
        }
        closedSet.insert(current.state);
        ++numOfStateExpansions;

        std::vector<Node> successors = generateSuccessors(current);
        for (auto &successor : successors) {
            if (closedSet.find(successor.state) == closedSet.end()) {
                openQueue.push(successor);
            } else {
                ++numDeletions;
            }
        }
    }

    actualRunningTime = float(clock() - start) / CLOCKS_PER_SEC;
    return "";
}

std::string aStarSearch(const std::string &initialState, const std::string &goalState,
                        int &pathLength, int &numOfStateExpansions, int &maxQLength, float &actualRunningTime,
                        std::function<int(const std::string&, const std::string&)> heuristic) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openQueue;
    std::set<std::string> closedSet;
    int numDeletions = 0;

    clock_t start = clock();

    openQueue.emplace(initialState, "", 0, heuristic(initialState, goalState));
    maxQLength = 0;
    numOfStateExpansions = 0;

    while (!openQueue.empty()) {
        maxQLength = std::max(maxQLength, (int)openQueue.size());

        Node current = openQueue.top();
        openQueue.pop();

        if (current.state == goalState) {
            pathLength = current.path.length();
            actualRunningTime = float(clock() - start) / CLOCKS_PER_SEC;
            return current.path;
        }

        if (closedSet.find(current.state) != closedSet.end()) {
            continue;
        }
        closedSet.insert(current.state);
        ++numOfStateExpansions;

        std::vector<Node> successors = generateSuccessors(current);
        for (auto &successor : successors) {
            successor.heuristic = heuristic(successor.state, goalState);
            if (closedSet.find(successor.state) == closedSet.end()) {
                openQueue.push(successor);
            } else {
                ++numDeletions;
            }
        }
    }

    actualRunningTime = float(clock() - start) / CLOCKS_PER_SEC;
    return "";
}

int main() {
    std::string startState;
    const std::string goalState = "123456780";

    std::cout << "Enter the initial state (9 characters, with '0' for blank tile): ";
    std::cin >> startState;

    int pathLength, numOfStateExpansions, maxQLength;
    float actualRunningTime;

    std::string ucsPath = uniformCostSearch(startState, goalState, pathLength, numOfStateExpansions, maxQLength, actualRunningTime);
    std::cout << "Uniform Cost Search:\n";
    std::cout << "Path: " << ucsPath << "\n";
    std::cout << "Path Length: " << pathLength << "\n";
    std::cout << "Number of State Expansions: " << numOfStateExpansions << "\n";
    std::cout << "Max Q Length: " << maxQLength << "\n";
    std::cout << "Actual Running Time: " << actualRunningTime << " seconds\n";

    std::string aStarPathMT = aStarSearch(startState, goalState, pathLength, numOfStateExpansions, maxQLength, actualRunningTime, misplacedTiles);
    std::cout << "\nA* Search (Misplaced Tiles):\n";
    std::cout << "Path: " << aStarPathMT << "\n";
    std::cout << "Path Length: " << pathLength << "\n";
    std::cout << "Number of State Expansions: " << numOfStateExpansions << "\n";
    std::cout << "Max Q Length: " << maxQLength << "\n";
    std::cout << "Actual Running Time: " << actualRunningTime << " seconds\n";

    std::string aStarPathMD = aStarSearch(startState, goalState, pathLength, numOfStateExpansions, maxQLength, actualRunningTime, manhattanDistance);
    std::cout << "\nA* Search (Manhattan Distance):\n";
    std::cout << "Path: " << aStarPathMD << "\n";
    std::cout << "Path Length: " << pathLength << "\n";
    std::cout << "Number of State Expansions: " << numOfStateExpansions << "\n";
    std::cout << "Max Q Length: " << maxQLength << "\n";
    std::cout << "Actual Running Time: " << actualRunningTime << " seconds\n";

    return 0;
}
