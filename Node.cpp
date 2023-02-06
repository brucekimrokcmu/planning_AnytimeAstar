#include "Node.hpp"
#include <bits/stdc++.h>
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

Node::Node()  
{    
};

Node::Node(int poseX, int poseY, int currTime, double* map, int xSize, int ySize) 
    : mX(poseX), mY(poseY), mT(currTime),
    mC((int)map[GETMAPINDEX(poseX, poseY, xSize, ySize)]), mG(INT_MAX), mH(0), 
    mbClosed(false), mbExpanded(false)
{        
};
// Pose, Time
int Node::GetPoseX() const {return mX;}
int Node::GetPoseY() const {return mY;}
int Node::GetCurrentTime() const {return mT;}

// Cost, G, H
double Node::GetCellCost() const {return mC;}
double Node::GetGValue() const {return mG;}
void Node::SetGValue(const double val)
{
    mG = val;
}
double Node::GetHeuristics() const {return mH;}
void Node::SetHeuristics(const double val) 
{
    mH = val;
}

// Boolean: Closed, Expanded
bool Node::GetBoolClosed() const {return mbClosed;}
void Node::SetBoolClosed(const bool val)
{
    mbClosed = val;
}
bool Node::GetBoolExpanded() const {return mbExpanded;}
void Node::SetBoolExpanded(const bool val)
{
    mbExpanded = val;
}
