#include "Node.hpp"
#include <limits>

Node::Node(int poseX, int poseY, int currTime) 
    : mX(poseX), mY(poseY), mT(currTime),
    mF(0), mG(std::numeric_limits<double>::infinity()), mH(0), mGoalCost(0),mpParent(nullptr)
    // mF(0), mG(9999.0), mH(0), mpParent(nullptr)
{        
};

Node::Node(int poseX, int poseY, int currTime, bool imGoal) 
    : mX(poseX), mY(poseY), mT(currTime), mIsImaginaryGoal(false),
    mF(0), mG(std::numeric_limits<double>::infinity()), mH(0), mGoalCost(0),mpParent(nullptr)
    // mF(0), mG(9999.0), mH(0), mpParent(nullptr)
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

double Node::GetGoalCost() const {return mGoalCost;}
void Node::SetGoalCost(const double val)
{
    mGoalCost = val;
}

Node* Node::GetParent() const
{
    return mpParent;
}

void Node::SetParent(Node* pParent)
{
    mpParent = pParent;
}