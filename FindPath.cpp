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
    double weight = 1.0;

    if (mPlanningFlag)
    {
        Node startNode(robotposeX, robotposeY, curr_time);    
        Node goalNode(targetposeX, targetposeY, curr_time);

        // Compute heuristics
        if (IsCellValid(startNode)) {
            startNode.SetGValue(0.0);
            startNode.SetHeuristics(ComputeEuclideanHeuristics(startNode, goalNode));
            startNode.SetFValue(ComputeFValue(startNode, weight));
        }

        if (IsCellValid(goalNode)){
            startNode.SetGValue(mmap[GetNodeIndex(goalNode)]);
            startNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
            startNode.SetFValue(ComputeFValue(goalNode, weight));
        }

        AStar(startNode, goalNode, curr_time, weight);
        // For AstarwithDijkstra, I will pass DikstraHeuristics as an argument
        // AStarwithDijkstra()
        std::vector<Node*> path = GetPath(goalNode);
        mPlanningFlag = false;
    } 

    

    // action_ptr[0] = nextRobotPoseX;
    // action_ptr[1] = nextRobotPoseY;
    return;
}



    // for (int dir=0; dir<NUMOFDIRS; dir++) {
    //     int newX = currNode->GetPoseX() + mdX[dir];
    //     int newY = currNode->GetPoseY() + mdY[dir];

void FindPath::AStar(Node startNode, Node goalNode, int currTime, double weight)
{
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;

    openList.push(&startNode); // OPEN = {s_start}; 

    // while(s_goal is not expanded && OPEN!=0){ 
    while((!IsVisited(closedList, GetNodeIndex(goalNode))) && (!openList.empty())) {
        //  remove s with the smallest [f(s)=g(s)+h(s)] from OPEN;
        //  insert s into CLOSED;
        Node* topNode = openList.top(); 
        closedList[GetNodeIndex(*topNode)] = topNode;
        openList.pop();
        
        //  for every successor s' of s such that s' not in CLOSED;  
        for (int dir=0; dir<NUMOFDIRS; dir++){ // need to calculate time, which increases by +1
            int newX = topNode->GetPoseX() + mdX[dir];
            int newY = topNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
            
            if (IsVisited(closedList, newIndex)){ 
                // If visited & g_value and heuristics should be updated, update and push to open list
                Node* existingNode = closedList[newIndex];
                if (existingNode->GetGValue() > topNode->GetGValue()+mmap[newIndex]){
                    existingNode->SetGValue(topNode->GetGValue()+mmap[newIndex]);
                    existingNode->SetHeuristics(ComputeEuclideanHeuristics(*existingNode, goalNode));
                    existingNode->SetFValue(ComputeFValue(*existingNode, weight));
                    existingNode->SetParent(topNode);
                    openList.push(existingNode); 
                }
            } else {
                // If not visited -> create a node
                Node succNode(newX, newY, currTime);
                succNode.SetHeuristics(ComputeEuclideanHeuristics(succNode, goalNode)); 
                // if g(s') > g(s) + c(s, s')
                //     g(s') = g(s) + c(s, s')
                //     insert s' into OPEN
                    if (succNode.GetGValue() > topNode->GetGValue()+mmap[newIndex]){
                        succNode.SetGValue(topNode->GetGValue()+mmap[newIndex]);
                        succNode.SetHeuristics(ComputeEuclideanHeuristics(succNode, goalNode));
                        succNode.SetFValue(ComputeFValue(succNode, weight));
                        succNode.SetParent(topNode);
                        openList.push(&succNode);
                    }
            }
        }
    } 

    return;
}

void AStarwithDijkstra(Node startNode, Node goalNode, int currTime)
{
}

std::vector<Node*> FindPath::GetPath(Node goalNode)
{
    std::vector<Node*> path;
    path.emplace_back(&goalNode);

    Node* pparent = goalNode.GetParent();
    while(pparent != nullptr){
        pparent = pparent->GetParent();
        path.emplace_back(pparent);
    }
    return  path;
}


int FindPath::GetNodeIndex(Node node)
{
    int x = node.GetPoseX();
    int y = node.GetPoseY();
    return ((y-1)*mxSize + (x-1));
}

int FindPath::GetIndexFromPose(int x, int y)
{
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

bool FindPath::IsVisited(std::unordered_map<int, Node*> list, int index)
{
    if (list.find(index) == list.end()) {return false;} 
    return true;
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

void FindPath::ComputeDijkstraHeuristics(Node currNode)
{
 // Implement Backward Dijkstra
 // while(!openList.empty()) 
    
    // mtargetTrajectory;

}
