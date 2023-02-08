#include "Node.hpp"
#include <limits>

Node::Node()  
{    
};

Node::Node(int poseX, int poseY, int currTime) 
    : mX(poseX), mY(poseY), mT(currTime),
    mF(0), mG(std::numeric_limits<double>::infinity()), mH(0), mpparent(nullptr)
    // mbClosed(false), mbExpanded(false)
{        
};
// Pose, Time
int Node::GetPoseX() const {return mX;}
int Node::GetPoseY() const {return mY;}
int Node::GetCurrentTime() const {return mT;}
void Node::SetCurrentTime(const int val) 
{
    mT = val;
}

// G, H, F
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
double Node::GetFValue() const {return mF;}
void Node::SetFValue(const double val)
{   
    mF = val;
}

void Node::SetParent(Node* pparent)
{
    mpparent = pparent;
}