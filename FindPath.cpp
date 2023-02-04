#include "FindPath.hpp"
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1)) // redundant define ?


int ComputeGValue(GraphHelper::Node n, GraphHelper::Node startNode,
const double* map, const int xSize, const int ySize)
{
    
    return 1;
}
int ComputeHeuristics(GraphHelper::Node n, GraphHelper::Node goalNode)
{
    return 1;
}
int ComputeCostBwNodes(GraphHelper::Node n1, GraphHelper::Node n2, const double* map, const int xSize, const int ySize)
{
    return 1;   
}
