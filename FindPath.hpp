#pragma once
#include "GraphHelper.hpp"

class FindPath{
    public:
        FindPath(); // constructor... initialize what?

        int ComputeGValue(GraphHelper::Node n, GraphHelper::Node startNode,
        const double* map, const int xSize, const int ySize);
        int ComputeHeuristics(GraphHelper::Node n, GraphHelper::Node goalNode);
        int ComputeCostBwNodes(GraphHelper::Node n1, GraphHelper::Node n2, const double* map, const int xSize, const int ySize);


};