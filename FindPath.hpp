#pragma once

#include <iostream>
#include <math.h>
#include <unordered_map>
#include <queue>
#include <vector>
#include "FValueCompare.hpp"
#include "Node.hpp"

#define NUMOFDIRS 8

class FindPath {
    public:
        FindPath();
        FindPath(double* map,
                 int collision_thresh,
                 int x_size,
                 int y_size,
                 int target_steps,
                 double* target_traj);
        
        static bool mPlanningFlag;
        static std::vector<Node*> mPath;
        static int mPathLength;
        static int mPathIterator;
        
        std::pair<int, int> Execute(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr);                    

        void AStar(Node startNode, Node goalNode, int currTime, double eps);
        // void AStarwithDijkstra(Node startNode, Node goalNode, int currTime);
        std::vector<Node*> GetPath(Node goalNode);

        int GetNodeIndex(Node node);
        int GetIndexFromPose(int x, int y);
        bool IsCellValid(Node node);
        bool IsVisited(std::unordered_map<int, Node*> list, int index);

        // double ComputeGValue(Node startNode, Node node, int currTime);
        double ComputeEuclideanHeuristics(Node node, Node goalNode);
        double ComputeFValue(Node node, double eps);
        void ComputeDijkstraHeuristics(Node currNode);

    private:
        double *mmap;
        int mcollisionThresh;
        int mxSize;
        int mySize;
        int mtargetSteps;
        double* mtargetTrajectory;
        int mdX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int mdY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1}; 

};