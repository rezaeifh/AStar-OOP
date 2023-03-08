#include <bits/stdc++.h>
#include "AStar.h"
using namespace std;

// Class constructor
Astar::Astar(vector<vector<int>> mgrid, vector<int> mstart, vector<int> mgoal){

    grid = mgrid;
    start = mstart;
    goal = mgoal;
    ROW = grid.size();
    COL = grid[0].size();
}


// A Utility Function to check whether given cell (row, col) is a valid cell or not.
// It returns true if the is inside the map.
bool Astar::isValid(int row, int col)
{

	return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is blocked or not.
// It returns true if the cell is not on obstacle cell.
bool Astar::isUnBlocked(vector<vector<int>> grid, int row, int col)
{

	if (grid[row][col] == 1)
		return (true);
	else
		return (false);
}

// A Utility Function to check whether goal cell has been reached or not.
bool Astar::isDestination(int row, int col, vector<int> goal)
{
	if (row == goal[0] && col == goal[1])
		return (true);
	else
		return (false);
}

// A Utility Function to trace the path from the start to goal and prepare the output of the code.
void Astar::tracePath(vector<vector<cell>> cellDetails, vector<int> goal, vector<vector<int>> grid)
{
	cout << endl;
    cout << "The Path is ";

	int row = goal[0];
	int col = goal[1];
    int ori = goal[2];

    // Define the goal cell as 3 on the map.
    grid[row][col] = 3;

    // A stack for gathering the trace.
	stack<vector<int>> Path;

	while (!(cellDetails[row][col].parent_i == row
			&& cellDetails[row][col].parent_j == col)) {

        // Define the trace cells as 4 on the map.
        if (row != goal[0] || col != goal[1]) grid[row][col] = 4;

        // Add the trace.
		Path.push({row, col, ori});
        if (ori != cellDetails[row][col].parent_o) Path.push({row, col, cellDetails[row][col].parent_o});
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
        int temp_ori = cellDetails[row][col].parent_o;
		row = temp_row;
		col = temp_col;
        ori = temp_ori;
	}

    // Add the trace of the start cell.
	Path.push({row, col, ori});
    if (ori != cellDetails[row][col].parent_o) Path.push({row, col, cellDetails[row][col].parent_o});

    // Define start cell as 2 on the map.
    grid[row][col] = 2;

    // Prepare output which shows the trace.
    // It shows the cells occupied with the robot and also the rotation of the robot.
	vector<int> p;
    string dir;
    int count = 0;

	while (!Path.empty()) {

		p = Path.top();
		Path.pop();
        if (p[2]==1) dir = " North";
        else if (p[2]==2) dir = " South";
        else if (p[2]==3) dir = " East";
        else if (p[2]==4) dir = " West";
		cout << "-> " << "(" << p[1]+1 << "," << ROW - p[0] << "," << dir << ") ";
        if (count == 10) {
            cout << endl;
            count = 0;
        }
        count++;
	}

    cout << endl;

    // Prepare the full map with obstacles, free spaces, start cell, goal cell, and trace.
    cout << "The trace of the path is shown below (0 is obstacle cells, 1 is unoccupied cells, 2 is the start cell, 3 is the goal cell, and 4 are cells between the start and goal):" << endl;
    for (int i = 0; i < ROW; i++){
        for (int j =0; j < COL; j++){

            cout << grid[i][j] << " ";
        }
        cout << endl;
    }

	return;
}


// A* Search Algorithm
void Astar::aStarSearch(vector<vector<int>> grid, vector<int> start, vector<int> goal)
{
	// If the start cell is out of range.
	if (isValid(start[0], start[1]) == false) {

		cout << "Start cell is invalid" << endl;
		return;
	}

	// If the goal cell is out of range.
	if (isValid(goal[0], goal[1]) == false) {

		cout << "Goal cell is invalid" << endl;
		return;
	}

	// Either the start or the goal is on obstacle
	if (isUnBlocked(grid, start[0], start[1]) == false
		|| isUnBlocked(grid, goal[0], goal[1])
			== false) {

		cout << "Start cell or the goal cell is blocked" << endl;
		return;
	}

	// If the goal cell is the same as source cell
	if (isDestination(start[0], start[1], goal)
		== true) {

		cout << "We are already at the destination" << endl;
		return;
	}

	// Create a closed list and initialize it to false which means that no cell has been included yet.
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D vector of structure to hold the details of the cell.
    vector<vector<cell>> cellDetails(ROW,vector<cell>(COL));

	int i, j,k;

	for (i = 0; i < ROW; i++) {
		for (j = 0; j < COL; j++) {
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
            cellDetails[i][j].parent_o = -1;
		}
	}

	// Initialize the parameters of the start node
	i = start[0], j = start[1], k = start[2];
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;
    cellDetails[i][j].parent_o = k;


	// Create an open list having information as- <f, <i, j, k>> where f is sum of g and h.
    // g = the movement cost to move from the starting point to a given square on the grid, following the path generated to get there.
    // h =  the estimated movement cost to move from that given square on the grid to the final destination.
	set<pPair> openList;

	// Put the start cell on the open list and set its f as 0
	vector<int> hold;
	hold = {i,j,k};
	openList.insert(make_pair(0.0, hold));

	// Set a boolean value as false initially and use it see whether the destination reached or not.
	bool foundDest = false;

	while (!openList.empty()) {

		pPair p = *openList.begin();

		// Remove this vertex from the open list
		openList.erase(openList.begin());

		// Add this vertex to the closed list
		i = p.second[0];
		j = p.second[1];
        k = p.second[2];
		closedList[i][j] = true;

		/*
		Generating all the 4 successor of this cell

		Cell-->Popped Cell (i, j)
		N --> North	(1) (i-1, j)
		S --> South	(2) (i+1, j)
		E --> East	(3) (i, j+1)
		W --> West  (4) (i, j-1)
        */

		// To store the 'g', 'h' and 'f' of the 4 successors
		double gNew, hNew, fNew;

		//1st Successor (North)
		if (isValid(i - 1, j) == true) {

			// If the goal cell is the same as the current successor
			if (isDestination(i - 1, j, goal) == true) {

				// Set the Parent of the goal cell
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
                cellDetails[i - 1][j].parent_o = 1;
				cout << "The goal cell is found" << endl;
				tracePath(cellDetails, goal, grid);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed list or if it is blocked, then ignore it.
			else if (closedList[i - 1][j] == false
					&& isUnBlocked(grid, i - 1, j)
							== true) {

				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, goal);
				fNew = gNew + hNew;

				// If the successor isn’t on the open list, add it to the open list.
                // Make the current square the parent of this square. Record the
				// f, g, and h costs of the square cell
				// OR
				// If it is on the open list already, check to see if this path to that
                // square is better, using 'f' cost as the measure.
				if (cellDetails[i - 1][j].f == FLT_MAX
					|| cellDetails[i - 1][j].f > fNew) {

					    hold = {i - 1, j, 1};
					openList.insert(make_pair(
						fNew, hold));

					// Update the details of this cell
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
                    cellDetails[i - 1][j].parent_o = 1;
				}
			}
		}

		// 2nd Successor (South)
		if (isValid(i + 1, j) == true) {

			// If the goal cell is the same as the current successor
			if (isDestination(i + 1, j, goal) == true) {

				// Set the Parent of the goal cell
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
                cellDetails[i + 1][j].parent_o = 2;
				cout << "The goal cell is found" << endl;
				tracePath(cellDetails, goal, grid);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed list or if it is blocked, then ignore it.
			else if (closedList[i + 1][j] == false
					&& isUnBlocked(grid, i + 1, j)
							== true) {

				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, goal);
				fNew = gNew + hNew;

                // If the successor isn’t on the open list, add it to the open list.
                // Make the current square the parent of this square. Record the
                // f, g, and h costs of the square cell
                // OR
                // If it is on the open list already, check to see if this path to that
                // square is better, using 'f' cost as the measure.
				if (cellDetails[i + 1][j].f == FLT_MAX
					|| cellDetails[i + 1][j].f > fNew) {

					    hold = {i + 1, j, 2};
					openList.insert(make_pair(
						fNew, hold));

					// Update the details of this cell
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
                    cellDetails[i + 1][j].parent_o = 2;
				}
			}
		}

		// 3rd Successor (East)
		if (isValid(i, j + 1) == true) {

			// If the goal cell is the same as the current successor
			if (isDestination(i, j + 1, goal) == true) {

				// Set the Parent of the goal cell
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
                cellDetails[i][j + 1].parent_o = 3;
				cout << "The goal cell is found" << endl;
				tracePath(cellDetails, goal, grid);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed list or if it is blocked, then ignore it.
			else if (closedList[i][j + 1] == false
					&& isUnBlocked(grid, i, j + 1)
							== true) {

				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, goal);
				fNew = gNew + hNew;

                // If the successor isn’t on the open list, add it to the open list.
                // Make the current square the parent of this square. Record the
                // f, g, and h costs of the square cell
                // OR
                // If it is on the open list already, check to see if this path to that
                // square is better, using 'f' cost as the measure.
				if (cellDetails[i][j + 1].f == FLT_MAX
					|| cellDetails[i][j + 1].f > fNew) {

					    hold = {i, j+1, 3};
					openList.insert(make_pair(
						fNew, hold));

					// Update the details of this cell
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
                    cellDetails[i][j + 1].parent_o = 3;
				}
			}
		}

		// 4th Successor (West)
		if (isValid(i, j - 1) == true) {

			// If the destination cell is the same as the current successor
			if (isDestination(i, j - 1, goal) == true) {
				// Set the Parent of the destination cell
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
                cellDetails[i][j - 1].parent_o = 4;
				cout << "The goal cell is found" << endl;
				tracePath(cellDetails, goal, grid);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed list or if it is blocked, then ignore it.
			else if (closedList[i][j - 1] == false
					&& isUnBlocked(grid, i, j - 1)
							== true) {

				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, goal);
				fNew = gNew + hNew;

                // If the successor isn’t on the open list, add it to the open list.
                // Make the current square the parent of this square. Record the
                // f, g, and h costs of the square cell
                // OR
                // If it is on the open list already, check to see if this path to that
                // square is better, using 'f' cost as the measure.
				if (cellDetails[i][j - 1].f == FLT_MAX
					|| cellDetails[i][j - 1].f > fNew) {

					    hold = {i, j - 1, 4};
					openList.insert(make_pair(
						fNew, hold));

					// Update the details of this cell
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
                    cellDetails[i][j - 1].parent_o = 4;
				}
			}
		}
	}

	// When the goal cell is not found and the open list is empty, then we conclude that we failed to reach the goal cell.
	if (foundDest == false)
		cout << "Failed to find the goal Cell" << endl;

	return;
}