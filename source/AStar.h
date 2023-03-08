#ifndef AStar_H
#define AStar_H

#include <vector>

using namespace std;

// Create a shortcut for pair<double, vector<int>> type
typedef pair<double, vector<int>> pPair;

// A structure to keep the necessary parameters of a cell
struct cell {
	int parent_i, parent_j, parent_o;
	double f, g, h;
};

// Abstract Class
class Astar
{

public:
    int ROW, COL;
    vector<vector<int>> grid;
    vector<int> start, goal;

    Astar(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal);

    bool isValid(int row, int col);
    bool isUnBlocked(vector<vector<int>> grid, int row, int col);
    bool isDestination(int row, int col, vector<int> goal);
    virtual double calculateHValue(int row, int col, vector<int> goal) = 0;
    void tracePath(vector<vector<cell>> cellDetails, vector<int> goal, vector<vector<int>> mgrid);
    void aStarSearch(vector<vector<int>> grid, vector<int> start, vector<int> goal);
};

// Euclidean class that inherits Astar
class Euclidean : public Astar {
public:
    Euclidean(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal) : Astar(mgrid,mstart,mgoal){};
    double calculateHValue(int row, int col, vector<int> goal);
};

// Manhattan class that inherits Astar
class Manhattan : public Astar {
public:
    Manhattan(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal) : Astar(mgrid,mstart,mgoal){};
    double calculateHValue(int row, int col, vector<int> goal);
};

// Octagonal class that inherits Astar
class Octagonal : public Astar {
public:
    Octagonal(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal) : Astar(mgrid,mstart,mgoal){};
    double calculateHValue(int row, int col, vector<int> goal);
};

// Diagonal class that inherits Astar
class Diagonal : public Astar {
public:
    Diagonal(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal) : Astar(mgrid,mstart,mgoal){};
    double calculateHValue(int row, int col, vector<int> goal);
};

#endif
