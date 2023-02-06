#pragma once
#include "Node.hpp"

class FindPath{
    public:
        // FindPath(); 

        int ComputeCostBwNodes(Node node1, Node node2, const double* map, const int xSize, const int ySize);

        // int ComputeGValue(Node node, Node startNode,
        // const double* map, const int xSize, const int ySize);
        
        int ComputeHeuristics(Node node, Node goalNode);


}