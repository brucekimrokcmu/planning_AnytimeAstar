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
    
    // action_ptr[0] = nextRobotPoseX;
    // action_ptr[1] = nextRobotPoseY;
    return;
}

std::vector<Node> FindPath::CreateSmallGraph(Node currNode, int currTime)
{
    std::vector<Node> smallGraph;
    for (int dir=0; dir<NUMOFDIRS; dir++) {
        int newX = currNode.GetPoseX() + mdX[dir];
        int newY = currNode.GetPoseY() + mdY[dir];
        
        Node newNode(newX, newY, currTime);

        if(IsCellValid(newNode)) {
            // update gValue of newNode
            // update heuristics of newNode
            // update FValue of newNode

            smallGraph.push_back(newNode);
        }

    }


    return smallGraph;
}


void FindPath::AStar(Node startNode, Node goalNode)
{

    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    openList.push(&startNode); // OPEN = {s_start}; 

    // while(s_goal is not expanded && OPEN!=0){ 
    while((!goalNode.GetBoolExpanded()) && (!openList.empty())) {
        
    
    // remove s with the smallest [f(s)=g(s)+h(s)] from OPEN;
    //  insert s into CLOSED;
        Node* topNode = openList.top(); 
        topNode->SetBoolClosed(true);
        openList.pop();
        //  might want to try reference
        

        

    //  for every successor s' of s such that s' not in CLOSED;
        for (int i=0; i<smallGraph.size(); i++){
            Node n_ref = smallGraph[i]

        }
    //     if g(s') > g(s) + c(s, s')


//             g(s') = g(s) + c(s, s')
//             insert s' into OPEN;

    } 

}

int FindPath::GetCellCost(Node node)
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
        int index = GetCellCost(node);
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