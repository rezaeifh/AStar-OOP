#include <bits/stdc++.h>
#include "AStar.h"

// Define Manhattan distance
double Manhattan::calculateHValue(int row, int col, vector<int> goal)
{
    // Return using the distance formula
    return ((double)abs(row - goal[0])
            + abs(col - goal[1]));
}
