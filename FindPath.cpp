#include "FindPath.hpp"
FindPath::FindPath(){};
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

std::pair<int, int> FindPath::Execute(int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr)
{
    // double weight = 1.0;

    // if (mPlanningFlag)
    // {
        
    //     Node startNode(robotposeX, robotposeY, curr_time);  
    //     // this doesn't take account of goalpose changes w.r.t. time changes  
    //     // Let's first assume that goal pose is stationary and try to debug
    //     Node goalNode(targetposeX, targetposeY, curr_time); 

    //     // Compute heuristics
    //     if (IsCellValid(startNode)) {
    //         startNode.SetGValue(0.0);
    //         startNode.SetHeuristics(ComputeEuclideanHeuristics(startNode, goalNode));
    //         startNode.SetFValue(ComputeFValue(startNode, weight));
    //     }

    //     if (IsCellValid(goalNode)){
    //         startNode.SetGValue(mmap[GetNodeIndex(goalNode)]);
    //         startNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
    //         startNode.SetFValue(ComputeFValue(goalNode, weight));
    //     }

    //     AStar(startNode, goalNode, curr_time, weight);
    //     // For AstarwithDijkstra, I will pass DikstraHeuristics as an argument
    //     // AStarwithDijkstra()

    //     Node* pgoalNode = &goalNode;
    //     mPath = GetPath(pgoalNode);
    //     mPathLength = mPath.size();
    //     mPathIterator = 0;
    //     mPlanningFlag = false;
    
    // } 
    // // now need to think about time;
    // int nextRobotPoseX;
    // int nextRobotPoseY;

    // if (mPathIterator < mPathLength){
    //     nextRobotPoseX = mPath[mPathIterator]->GetPoseX();
    //     nextRobotPoseY = mPath[mPathIterator]->GetPoseY();
    //     mPathIterator++;
    // }
    

    std::pair<int, int> nextPose;
    nextPose.first = 1;//nextRobotPoseX;
    nextPose.second =1;// nextRobotPoseY;

    // curr_time++;

    return nextPose;
}

void FindPath::AStar(Node* pstartNode, Node* pgoalNode, int currTime, double weight)
{
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    
    // OPEN = {s_start}; 
    openList.push(pstartNode); 
    // while(s_goal is not expanded && OPEN!=0){ 
    while((closedList.find(GetNodeIndex(pgoalNode)) == closedList.end()) && (!openList.empty())) {
        
        //  remove s with the smallest [f(s)=g(s)+h(s)] from OPEN;
        Node* topNode = openList.top();  
        openList.pop();
        //  insert s into CLOSED;
        closedList[GetNodeIndex(topNode)] = topNode;
        
        int newX = topNode->GetPoseX();
        int newY = topNode->GetPoseY();
        //  for every successor s' of s such that s' not in CLOSED;  
        for (int dir=0; dir<NUMOFDIRS; dir++){ // need to calculate time, which increases by +1
            newX += mdX[dir];
            newY += mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
            Node succNode(newX, newY, currTime);
            Node *psuccNode = &succNode;
            // printf("succnode pose: %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY());
            psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, pgoalNode));

            if (closedList.find(newIndex) == closedList.end()){ // If NOT visited    
                // printf("Not Visited.\n");
                if (psuccNode->GetGValue()> topNode->GetGValue()+mmap[newIndex]){
                    psuccNode->SetGValue(topNode->GetGValue()+mmap[newIndex]);
                    psuccNode->SetFValue(ComputeFValue(psuccNode, weight));                
                    psuccNode->SetParent(topNode);
                    // double succNode_Fval = psuccNode->GetFValue();
                    // printf("succNodeFval: %f;\n", succNode_Fval);
                    openList.push(psuccNode);
                    double fvalue = openList.top()->GetFValue();
                    printf("opentop fval: %f;\n", fvalue);
                    // int size = openList.size();
                    // printf("openlist size is: %d;\n", size);
                }    
            } else { // If visited
                // printf("Already visited.\n");
                Node* pexistingNode = closedList[newIndex];
                if(pexistingNode->GetFValue() > mmap[newIndex]+weight*psuccNode->GetHeuristics()){
                    closedList[newIndex] = psuccNode;
                    psuccNode->SetParent(topNode);
                    openList.push(psuccNode);
                    // double fvalue = openList.top()->GetFValue();
                    // printf("existing opentop fval: %f;\n", fvalue);
                    // printf("Pushed to open list.***\n\n");   
                }
            }
        }
        
        // while(!openList.empty()){
        //     int fvalue = openList.top()->GetFValue();
        //     printf("fval: %d;\n", fvalue);
        //     openList.pop();
        // }

    } 
    // printf("Size of closed list: %d;\n", closedList.size());
    // printf("Astar Complete\n");
    return;
}

// void FindPath::AStarwithDijkstra(Node startNode, Node goalNode, int currTime)
// {
// }

std::vector<Node*> FindPath::GetPath(Node* pgoalNode)
{
    std::vector<Node*> path;

    path.emplace_back(pgoalNode);

    Node* pparent = pgoalNode->GetParent();
    while(pparent != nullptr){
        pparent = pparent->GetParent();
        path.emplace_back(pparent);
    }
    std::reverse(path.begin(), path.end());
    return path;
}


int FindPath::GetNodeIndex(Node* pnode)
{
    // if(!IsCellValid(node)){
    //     return
    // }
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
    return ((y-1)*mxSize + (x-1));
}

int FindPath::GetIndexFromPose(int x, int y)
{
    return ((y-1)*mxSize + (x-1));
}

bool FindPath::IsCellValid(Node* pnode)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();

    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {
        int index = GetIndexFromPose(x,y);
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

double FindPath::ComputeEuclideanHeuristics(Node* pnode, Node* pgoalNode)
{
    int goalX = pgoalNode->GetPoseX();
    int goalY = pgoalNode->GetPoseY();
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();

    double euclideanHeuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return euclideanHeuristics;
}

double FindPath::ComputeFValue(Node* pnode, double eps)
{
    double fValue = pnode->GetGValue() + pnode->GetHeuristics()*eps;
    
    return fValue;
}

// for 3D
// solve 2D(x,y) and use that as heuristics for higher dimension
// so you would use Dijkstra at 2D and use that as heuristics for 3D

void FindPath::ComputeDijkstraHeuristics(Node* pcurrNode)
{
 // Implement Backward Dijkstra
 // while(!openList.empty()) 
    
    // mtargetTrajectory;

}
