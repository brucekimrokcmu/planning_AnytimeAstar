// #pragma once
#ifndef GRAPHHELPER_H_
#define GRAPHHELPER_H_
#include <vector>
#include <bits/stdc++.h>

class GraphHelper{
    public:
        GraphHelper(); //initialize what? what should be the member variables & functions?

        class Node{
            public:
                Node(int poseX, int poseY, int currTime) 
                : mX(poseX), mY(poseY), mT(currTime),
                  mG(INT_MAX), mH(0), 
                  mbStart(0), mbGoal(0) 
                {};
                
                const int GetPoseX() const;
                const int GetPoseY() const;
                const int GetCurrentTime() const;

                int GetGValue() const;
                void SetGValue(const int val);
                int GetHeuristics() const;
                void SetHeuristics(const int val);

                const bool GetBoolStart() const;
                void SetBoolStart(const bool state);
                const bool GetBoolGoal() const;
                void SetBoolGoal(const bool state);

            private:
                // a 3D state of mx, my, mt
                int mX;
                int mY;
                int mT;
                // g-value and heuristics
                int mG;
                int mH;
                // S_start, S_goal, or else
                bool mbStart;
                bool mbGoal;            
        };

        // Do I need GetStartNode and GetGoalNode?  
        
        Node GetNode(int posX, int posY, int currTime);

        // a general 8-grid graph generator surrounding the given node
        std::vector<GraphHelper::Node> CreateSmallGraph(Node& n,
        const double* map, const int collisionThresh, const int xSize, const int ySize, 
        int currTime     
        ); 

    // private:
    //     int x;

};

#endif


