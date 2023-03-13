#include <bits/stdc++.h>
#include "AStar.h"

// Define Octagonal distance
double Octagonal::calculateHValue(int row, int col, vector<int> goal)
{
    return ((double) 0.41 * abs(row - goal[0]) + 0.941246* abs(col - goal[1]));
}