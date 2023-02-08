#pragma once

#include <iostream>
#include <math.h>
#include <queue>
#include <vector>
#include "FValueCompare.hpp"
#include "Node.hpp"

#define NUMOFDIRS 9

class FindPath {
    public:
        FindPath(double* map,
                 int collision_thresh,
                 int x_size,
                 int y_size,
                 int target_steps,
                 double* target_traj);
        
        void Execute(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr);
                            
        std::vector<Node> CreateSmallGraph(Node* currNode, int currTime);
        void AStar(Node startNode, Node goalNode, int currTime);
        
        int GetNodeIndex(Node node);
        bool IsCellValid(Node node);
        // double ComputeGValue(Node startNode, Node node, int currTime);
        double ComputeEuclideanHeuristics(Node node, Node goalNode);
        double ComputeFValue(Node node, double eps);
        void ComputeDijkstra(Node goalNode, Node currNode);


    private:
        double *mmap;
        int mcollisionThresh;
        int mxSize;
        int mySize;
        int mtargetSteps;
        double* mtargetTrajectory;
        int mdX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  0,  1, 1, 1};
        int mdY[NUMOFDIRS] = {-1,  0,  1, -1,  0,  1, -1, 0, 1}; 


    


};