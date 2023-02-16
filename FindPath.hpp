#pragma once

#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <math.h>
#include <memory>
#include <unordered_map>
#include <queue>
#include <stack>
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
                 double* target_traj
                 );
        

        
        std::pair<int, int> ExecuteAStar(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr);                    


        std::vector<std::pair<int, int>> ExecuteMultigoalAStar(
                     int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr
                     );



        std::vector<std::pair<int, int>> ExecuteMultigoalAstarWithDijkstraHeuristics(
                     int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr,
                     std::unordered_map<int, double>* pg_DHueristics,
                     int target_time                     
                     );

        std::unordered_map<int, double> ComputeBackwardDijkstra();
    private:
        // For map information
        double *mmap;
        int mcollisionThresh;
        int mxSize;
        int mySize;
        int mtargetSteps;
        double* mtargetTrajectory;
        int mdX[NUMOFDIRS] = {-1, -1, -1,  0, 0,  1, 1, 1};
        int mdY[NUMOFDIRS] = {-1,  0,  1, -1, 1, -1, 0, 1}; 

        bool mPlanningFlag;
        int mPathLength;
        int mPathIterator;

        // std::unordered_map<int, double> mDHeuristics;


        // Helper member variables such as flag, graph, path
        static std::unique_ptr<std::unordered_map<int, Node>> mpFullGraph;
        static std::unique_ptr<std::vector<Node*>> mpOptimalPath;

        bool GetPlanningFlag()const {return mPlanningFlag;};
        bool GetPathLength()const {return mPathLength;};
        bool GetPathIterator()const {return mPathIterator;};

        void SetPlanningFlag(bool val) {mPlanningFlag = val;};
        void SetPathLength(int val) {mPathLength = val;};
        void IncreasePathIterator() {mPathIterator++;};

        // Algorithms
        std::vector<std::pair<int, int>> AStar(Node startNode, Node goalNode, int currTime);
        std::vector<std::pair<int, int>> AStarwithMultiBackwardDijkstra(Node startNode, int currTime, std::unordered_map<int, double>* pg_DHueristics, int targetTime);        
        std::vector<std::pair<int, int>> MultigoalAStar(Node startNode, int currTime, int targetTime);
        

        std::vector<Node*> GetOptimalPath(Node* pgoalNode);
        int GetNodeIndex(Node node); //CHECKED
        int GetNodeIndex(Node* pnode);
        int GetIndexFromPose(int x, int y); //CHECKED
        bool IsCellValid(Node node); //CHECKED
        bool IsCellValid(Node* pnode); 


        bool inline inBounds(int x, int y);
        bool inline isCollisionFree(int idx);



        double ComputeEuclideanHeuristics(Node node, Node goalNode); //CHECKED
        double ComputeEuclideanHeuristics(Node* pnode, Node* pgoalNode); //CHECKED
        double ComputeFValue(double gValue, double heuristics, double weight); //CHECKED
        
        // int BackwardAStarforDijkstra(Node startNode, Node goalNode, int currTime);
};