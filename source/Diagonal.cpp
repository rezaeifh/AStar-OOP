#include <bits/stdc++.h>
#include "AStar.h"

// Define Diagonal distance
double Diagonal::calculateHValue(int row, int col, vector<int> goal)
{
    int dx,dy;
    dx = abs(row - goal[0]);
    dy = abs(col - goal[1]);

    // Distance formula
    return ((double)(dx+dy) +(sqrt(2) - 2) * min(dx,dy));
}
