#include "FindPath.hpp"

FindPath::FindPath(double* map,
                 int collision_thresh,
                 int x_size,
                 int y_size,
                 int target_steps,
                 double* target_traj)
: mmap(map), mcollisionThresh(collision_thresh), 
mxSize(x_size), mySize(y_size), mtargetSteps(target_steps),
mtargetTrajectory(target_traj)
{
};

void FindPath::Execute(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr)
{
    double eps = 1.0;
    Node startNode(robotposeX, robotposeY, curr_time);    
    Node goalNode(targetposeX, targetposeY, curr_time);


    if (IsCellValid(startNode)) {
        startNode.SetGValue(0.0);
        startNode.SetHeuristics(ComputeEuclideanHeuristics(startNode, goalNode));
        startNode.SetFValue(ComputeFValue(startNode, eps));
    }

    if (IsCellValid(goalNode)){
        startNode.SetGValue(GetCellCost(goalNode));
        startNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
        startNode.SetFValue(ComputeFValue(goalNode, eps));
    }

    AStar(startNode, goalNode, curr_time);


    // action_ptr[0] = nextRobotPoseX;
    // action_ptr[1] = nextRobotPoseY;
    return;
}

std::vector<Node> FindPath::CreateSmallGraph(Node* currNode, int currTime)
{
    std::vector<Node> smallGraph;
    for (int dir=0; dir<NUMOFDIRS; dir++) {
        int newX = currNode->GetPoseX() + mdX[dir];
        int newY = currNode->GetPoseY() + mdY[dir];
        
        //how to expand graph without duplicating visited nodes? 

        Node newNode(newX, newY, currTime); // what if it overlaps with the previously defined node?

        if(IsCellValid(newNode)) {
            // update gValue of newNode
            // update heuristics of newNode
            // update FValue of newNode

            smallGraph.push_back(newNode);
        }
    }

    return smallGraph;
}

void FindPath::AStar(Node startNode, Node goalNode, int currTime)
{
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    openList.push(&startNode); // OPEN = {s_start}; 


    // while(s_goal is not expanded && OPEN!=0){ 
    while((!goalNode.GetBoolExpanded()) && (!openList.empty())) {
        //  remove s with the smallest [f(s)=g(s)+h(s)] from OPEN;
        //  insert s into CLOSED;
        Node* topNode = openList.top(); 
        topNode->SetBoolClosed(true);
        openList.pop();

        //  for every successor s' of s such that s' not in CLOSED;
        std::vector<Node> smallGraph = CreateSmallGraph(topNode, currTime);
        for (int i=0; i<smallGraph.size(); i++){
            if(!smallGraph[i].GetBoolClosed()){
                // if g(s') > g(s) + c(s, s')
                //     g(s') = g(s) + c(s, s')
                //     insert s' into OPEN
                if (smallGraph[i].GetGValue() > topNode->GetGValue()+GetCellCost(smallGraph[i])){
                    smallGraph[i].SetGValue(topNode->GetGValue()+GetCellCost(smallGraph[i]));
                    
                    openList.push(&smallGraph[i]);
                }
            }

        }
    } 

}

int FindPath::GetNodeIndex(Node node)
{
    int x = node.GetPoseX();
    int y = node.GetPoseY();
    return ((y-1)*mxSize + (x-1));
}

bool FindPath::IsCellValid(Node node)
{
    int x = node.GetPoseX();
    int y = node.GetPoseY();

    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {
        int index = GetNodeIndex(node);
        if (((int)mmap[index] >= 0) && ((int)mmap[index] < mcollisionThresh)) {
            return true;
        }
    }
    return false;
}

// double FindPath::ComputeGValue(Node startNode, Node node, int currTime)
// {
//     double gValue;
//     return gValue;
// }

double FindPath::ComputeEuclideanHeuristics(Node node, Node goalNode)
{
    int goalX = goalNode.GetPoseX();
    int goalY = goalNode.GetPoseY();
    int x = node.GetPoseX();
    int y = node.GetPoseY();

    double heuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return heuristics;
}

double FindPath::ComputeFValue(Node node, double eps)
{
    double fValue = node.GetGValue() + node.GetHeuristics()*eps;
    
    return fValue;
}

// for 3D
// solve 2D(x,y) and use that as heuristics for higher dimension
// so you would use Dijkstra at 2D and use that as heuristics for 3D

void FindPath::ComputeDijkstra(Node goalNode, Node currNode)
{
 // Implement Backward Dijkstra
 // while(!openList.empty()) 

}
