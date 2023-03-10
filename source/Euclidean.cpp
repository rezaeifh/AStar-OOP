#include <bits/stdc++.h>
#include "AStar.h"

// Define Euclidean distance
double Euclidean::calculateHValue(int row, int col, vector<int> goal)
{
    return ((double)sqrt(
            (row - goal[0]) * (row - goal[0])
            + (col - goal[1]) * (col - goal[1])));
}
