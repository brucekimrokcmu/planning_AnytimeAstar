#include "GraphHelper.hpp"
#define NUMOFDIRS 8 // redundant define ?
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1)) // redundant define ?

const int GraphHelper::Node::GetPoseX() const {return mX;}
const int GraphHelper::Node::GetPoseY() const {return mY;}
const int GraphHelper::Node::GetCurrentTime() const {return mT;}
int GraphHelper::Node::GetGValue() const {return mG;}
void GraphHelper::Node::SetGValue(const int val)
{
    mG = val;
}
int GraphHelper::Node::GetHeuristics() const {return mH;}
void GraphHelper::Node::SetHeuristics(const int val) 
{
    mH = val;
}
const bool GraphHelper::Node::GetBoolStart() const {return mbStart;}
void GraphHelper::Node::SetBoolStart(const bool state) 
{
    mbStart = state;
}
const bool GraphHelper::Node::GetBoolGoal() const {return mbGoal;}
void GraphHelper::Node::SetBoolGoal(const bool state)
{
    mbGoal =state;
}

GraphHelper::Node GraphHelper::GetNode(int poseX, int poseY, int currTime)
{
    return Node(poseX, poseY, currTime);
}

std::vector<GraphHelper::Node> GraphHelper::CreateSmallGraph(GraphHelper::Node& n, 
    const double* map, const int collisionThresh, const int xSize, const int ySize, 
    int currTime) //is currTime redundant? 
{
    std::vector<GraphHelper::Node> small_graph;
    int const dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int const dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int newX = n.GetPoseX(); 
    int newY = n.GetPoseY();
    int newT = n.GetCurrentTime();

    // add the current robotpose as a state after checking conditions such as collision threshold and map boundary 
    if (newX >= 1 && newX <= xSize && newY >= 1 && newY <= ySize) {
        if (((int)map[GETMAPINDEX(newX,newY,xSize,ySize)] >= 0) && 
            ((int)map[GETMAPINDEX(newX,newY,xSize,ySize)] < collisionThresh)) {
            small_graph.insert(small_graph.begin(),n);
        } // else what? 
   
    } // else what? 
    

    // add surrounding nodes
    for (int dir=0; dir<NUMOFDIRS; dir++) {
        newX += dX[dir];
        newY += dX[dir];

        if (newX >= 1 && newX <= xSize && newY >= 1 && newY <= ySize) {
            if (((int)map[GETMAPINDEX(newX,newY,xSize,ySize)] >= 0) && 
                ((int)map[GETMAPINDEX(newX,newY,xSize,ySize)] < collisionThresh)) {
                // Get a new node here 
                GraphHelper::Node newNode = GraphHelper::GetNode(newX, newY, currTime); 
                small_graph.insert((small_graph.begin()+1)+dir, newNode);
            }
        }
    }
    

    // Do I want to assign g-value and heuristics and then return the array of class?
    // assign g_value and heuristics

    return small_graph; // consider different data structure?
}