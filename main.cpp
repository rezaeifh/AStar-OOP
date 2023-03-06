#include <iostream>
#include "source/AStar.h"
#include <vector>
using namespace std;

int main() {

    // Required variables
    int row, col, n_o, x_o, y_o, in_gs, h;
    vector<vector<int>> obs;
    vector<int> start(3,0);
    vector<int> goal(3,0);

    // User defines the number of rows and columns
    cout << "Please type the number of rows: ";
    cin >> row;
    cout << "Please type the number of columns: ";
    cin >> col;

    // User defines the heuristics method
    cout << "Please determine the heuristics method with an int from 1 to 4 (1.Euclidean, 2.Manhattan, 3.Diagonal, 4.Octagonal): ";
    cin >> h;

    // User determines the location of the obstacle
    cout << "Please determine the number of cells acts as obstacles: ";
    cin >> n_o;
    cout << "Please enter the location of each obstacle:" << endl;

    for (int i = 0; i < n_o; i++) {

        cout << "x" << i+1 << ": ";
        cin >> y_o;
        cout << "y" << i+1 << ": ";
        cin >> x_o;
        obs.push_back({row-x_o,y_o-1});
    }

    // User determine the Start Cell
    cout << "Please determine the location of the start Cell with integers {Sx, Sy, Syaw(1. North, 2. South, 3. East, 4. West)}: "<< endl;
    for (int i = 0; i < 3; i++) {

        cin >> in_gs;
        if (i == 0) start[1] = in_gs-1;
        else if (i == 1) start[0] = row-in_gs;
        else start[2] = in_gs;
    }

    // Determine Goal Cell
    cout << "Please determine the location of the goal Cell with integers {Gx, Gy, Gyaw(1. North, 2. South, 3. East, 4. West)}: " << endl;
    for (int i = 0; i < 3; i++) {

        cin >> in_gs;
        if (i == 0) goal[1] = in_gs-1;
        else if (i == 1) goal[0] = row-in_gs;
        else goal[2] = in_gs;
    }

    // Define the map
    vector<vector<int>> grid(row,vector<int>(col,1));

    for (int i = 0; i < n_o; i++){

        grid[obs[i][0]][obs[i][1]] = 0;
    }

    // Define the object based on user preference (heuristics method)
    if (h == 2) {
        Manhattan train(grid,start,goal);
        train.aStarSearch(grid, start, goal);
    }
    else if (h == 3) {
        Diagonal train(grid,start,goal);
        train.aStarSearch(grid, start, goal);
    }
    else if (h == 4) {
        Octagonal train(grid,start,goal);
        train.aStarSearch(grid, start, goal);
    }
    else {
        Euclidean train(grid,start,goal);
        train.aStarSearch(grid, start, goal);
    }

	return (0);
}
