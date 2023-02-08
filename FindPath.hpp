#pragma once

#include <iostream>
#include <math.h>
#include <unordered_map>
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
                            

        void AStar(Node startNode, Node goalNode, int currTime, double eps);
        // void AStarwithDijkstra(Node startNode, Node goalNode, int currTime);
        int GetNodeIndex(Node node);
        int GetIndexFromPose(int x, int y);
        bool IsCellValid(Node node);
        bool IsVisited(std::unordered_map<int, Node*> list, int index);

        // double ComputeGValue(Node startNode, Node node, int currTime);
        double ComputeEuclideanHeuristics(Node node, Node goalNode);
        double ComputeFValue(Node node, double eps);
        void ComputeDijkstra(Node currNode);


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