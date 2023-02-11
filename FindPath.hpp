#pragma once

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <unordered_map>
#include <queue>
#include <vector>
#include "FValueCompare.hpp"
#include "Node.hpp"

#define NUMOFDIRS 9

class FindPath {
    public:
        FindPath();
        FindPath(double* map,
                 int collision_thresh,
                 int x_size,
                 int y_size,
                 int target_steps,
                 double* target_traj);
        
        bool mPlanningFlag;
        std::vector<Node*> mPath;
        int mPathLength;
        int mPathIterator;
        
        std::pair<int, int> Execute(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr);                    

        void AStar(Node* pstartNode, Node* pgoalNode, int currTime, double eps);
        // void AStarwithDijkstra(Node startNode, Node goalNode, int currTime);
        std::vector<Node*> GetPath(Node* pgoalNode);

        int GetNodeIndex(Node* pnode); //CHECKED
        int GetIndexFromPose(int x, int y); //CHECKED
        bool IsCellValid(Node* pnode); //CHECKED

        // double ComputeGValue(Node startNode, Node node, int currTime);
        double ComputeEuclideanHeuristics(Node* pnode, Node* pgoalNode); //CHECKED
        double ComputeFValue(double gValue, double heuristics, double weight); //CHECKED
        void ComputeDijkstraHeuristics(Node* pcurrNode); 

    private:
        double *mmap;
        int mcollisionThresh;
        int mxSize;
        int mySize;
        int mtargetSteps;
        double* mtargetTrajectory;
        int mdX[NUMOFDIRS] = {-1, -1, -1,  0, 0,  0,  1, 1, 1};
        int mdY[NUMOFDIRS] = {-1,  0,  1, -1, 0,  1, -1, 0, 1}; 

};