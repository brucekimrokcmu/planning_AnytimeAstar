#pragma once
#ifndef GRAPHHELPER_H_
#define GRAPHHELPER_H_
#include <vector>

class GraphHelper{
    public:
        GraphHelper();

        class Node{
            public:
                Node(int robotPoseX, int robotPoseY, int currTime) 
                : mx(robotPoseX), my(robotPoseY), mt(currTime), mbStart(0), mbOpened(0), mbClosed(0) 
                {};
                
                const int GetPoseX() const;
                const int GetPoseY() const;
                const int GetCurrentTime() const;
                const bool GetBoolStart() const;
                void SetBoolStart(const bool state);
                const bool GetBoolOpened() const;
                void SetBoolOpened(const bool state);
                const bool GetBoolClosed() const;
                void SetBoolClosed(const bool state);

            private:
                int mx;
                int my;
                int mt;
                bool mbStart;  
                bool mbOpened; // open:1,
                bool mbClosed;
                
        };
    
        int GetCellCost(Node n, double* map, int x_size, int y_size) const; // returns the map_cost_value

        std::vector<GraphHelper::Node> CreateSmallGraph(Node n, int robotposeX, int robotposeY,
        const double* map, const int collision_thresh, const int x_size, const int y_size, 
        int curr_time     
        ); 

    // private:
    //     int x;


};

#endif


