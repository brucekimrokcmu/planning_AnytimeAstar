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
mtargetTrajectory(target_traj), mPlanningFlag(true), mPathLength(0), mPathIterator(0)
{
};

std::pair<int, int> FindPath::Execute(
                     int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr)
{
    double weight = 1.0;
    Node startNode(robotposeX, robotposeY, curr_time);  
    // this doesn't take account of goalpose changes w.r.t. time changes  
    // Let's first assume that goal pose is stationary and try to debug
    Node goalNode(targetposeX, targetposeY, curr_time); 

    // Compute heuristics
    if (IsCellValid(startNode)) {
        startNode.SetGValue(0.0);
        startNode.SetHeuristics(ComputeEuclideanHeuristics(startNode, goalNode));
        startNode.SetFValue(ComputeFValue(startNode.GetGValue(), startNode.GetHeuristics(), weight));
    }
    if (IsCellValid(goalNode)){
        goalNode.SetGValue(mmap[GetNodeIndex(goalNode)]);
        goalNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
        goalNode.SetFValue(ComputeFValue(goalNode.GetGValue(),goalNode.GetHeuristics(), weight));
    }


    AStar(startNode, goalNode, curr_time);
    // For AstarwithDijkstra, I will pass DikstraHeuristics as an argument
    // AStarwithDijkstra()

    // Node* pgoalNode = &goalNode;
    // std::vector<Node*> mPath = GetOptimalPath(pgoalNode);
    
    // SetPathLength(mPath.size());
    // SetPlanningFlag(false);



    std::pair<int, int> nextPose;
    
    return nextPose;
}

// void FindPath::AStarwithDijkstra(Node startNode, Node goalNode, int currTime)
// {
// }

std::pair<int, int> FindPath::AStar(Node startNode, Node goalNode, int currTime)
{
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;

    double weight = 1.0;
    // mpFullGraph->insert({GetNodeIndex(startNode),&startNode});
    
    Node* pstartNode = new Node(startNode);

    openList.push(pstartNode); 
    visitedList[GetNodeIndex(pstartNode)] = pstartNode;
    

    // printf("pstart G H F %f %f %f;\n", pstartNode->GetGValue(), pstartNode->GetHeuristics(), pstartNode->GetFValue());
    
    while((closedList.find(GetNodeIndex(goalNode)) == closedList.end()) && (!openList.empty())) {
        Node* pparentNode = openList.top();  
        printf("\n\n\nparent address: %p;\n", pparentNode);
        printf("parentNode G H F: %f %f %f;\n", pparentNode->GetGValue(), pparentNode->GetHeuristics(), pparentNode->GetFValue());
        printf("parent pose: %d %d;\n", pparentNode->GetPoseX(), pparentNode->GetPoseY());
        openList.pop();
        closedList[GetNodeIndex(*pparentNode)] = pparentNode;
        //  for every successor s' of s such that s' not in CLOSED;  
        for (int dir=0; dir<NUMOFDIRS; dir++){ // need to calculate time, which increases by +1
            
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
            
            Node* psuccNode = new Node(newX, newY, currTime);
            if (IsCellValid(psuccNode)) {
                printf("Cell is Valid.\n");
                printf("succnode pose: %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY());
                psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, &goalNode));
                // printf("succnode Euclidean: %f;\n", psuccNode->GetHeuristics());
                
                if (visitedList.find(newIndex) != visitedList.end()){ // If visited 
                    printf("Visited already\n");    
                    if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
                    Node* pexistingNode = closedList.at(newIndex);
                        // printf("Already visited.\n");
                        if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                            printf("BEFORE psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
                            psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                            psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                            psuccNode->SetParent(pparentNode);
                            printf("AFTER psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
                            // closedList[newIndex] = psuccNode;
                            openList.push(psuccNode);
                            printf("Pushed to open list.***\n");   
                        }      
                    } 
                    
                } else { // If NOT visited 
                    printf("Not visited\n");
                    visitedList[newIndex]=psuccNode;

                    if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){
                        printf("BEFORE psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
                        
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                        psuccNode->SetParent(pparentNode);
                        printf("AFTER psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
                        openList.push(psuccNode);
                        
                        // printf("succNodeFval: %f;\n", succNode_Fval);
                        // double fvalue = openList.top()->GetFValue();
                        // printf("opentop fval: %f;\n", fvalue);
                        // int size = openList.size();
                        // printf("succ address: %p;\n", psuccNode);
                        printf("Pushed to open list.***\n"); 

                    } 
                    
                }
            
            } 
            else {
                printf("Invalid Cell.\n");
                continue;
            }
        }

    }


    // loop through closedlist, visitelist, delete all elements 
    std::pair<int, int> nextPose;
   
    for (auto i=closedList.begin(); i != closedList.end();i++) {
        delete i->second;
    }

    while (!openList.empty()){
        delete openList.top();
        openList.pop();
    }


    return nextPose;
}

std::vector<Node*> FindPath::GetOptimalPath(Node* pgoalNode)
{
    std::vector<Node*> path;

    path.push_back(pgoalNode);

    Node* pparent = pgoalNode->GetParent();
    while(pparent != nullptr){
        pparent = pparent->GetParent();
        path.push_back(pparent);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

int FindPath::GetNodeIndex(Node node)
{
    // if(!IsCellValid(node)){
    //     return
    // }
    int x = node.GetPoseX();
    int y = node.GetPoseY();
    return ((y-1)*mxSize + (x-1));
}

int FindPath::GetNodeIndex(Node* pnode)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
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
    int index = GetIndexFromPose(x,y);
    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {    
        if (((int)mmap[index] >= 0) && ((int)mmap[index] < mcollisionThresh)) {
            return true;
        }
    }
    return false;
}

bool FindPath::IsCellValid(Node* pnode)
{
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();
    int index = GetIndexFromPose(x,y);
    if (x >= 1 && x <= mxSize && y >= 1 && y <= mySize) {    
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

    double euclideanHeuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return euclideanHeuristics;
}

double FindPath::ComputeEuclideanHeuristics(Node* pnode, Node* pgoalNode)
{
    int goalX = pgoalNode->GetPoseX();
    int goalY = pgoalNode->GetPoseY();
    int x = pnode->GetPoseX();
    int y = pnode->GetPoseY();

    double euclideanHeuristics = (double)std::sqrt(((goalX - x)*(goalX-x) + (goalY - y)*(goalY-y)));

    return euclideanHeuristics;
}


double FindPath::ComputeFValue(double gValue, double heuristics, double weight)
{
    double fValue = gValue + heuristics*weight;
    // printf("****************COMPUTE FVALUE FUNC\n");
    // printf("fval, Gval, Hval, w: %f %f %f %f;\n",fValue, pnode->GetGValue(), pnode->GetHeuristics(), weight);
    // printf("****************END LINE *************\n");

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








// void FindPath::AStar(Node* pstartNode, Node* pgoalNode, int currTime, double weight)
// {
//     std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
//     std::unordered_map<int, Node*> closedList;
//     std::unordered_map<int, Node*> visitedList;
    
//     openList.push(pstartNode); 
//     // printf("pstart G H F %f %f %f;\n", pstartNode->GetGValue(), pstartNode->GetHeuristics(), pstartNode->GetFValue());
    
//     while((closedList.find(GetNodeIndex(pgoalNode)) == closedList.end()) && (!openList.empty())) {
//         Node parentNode; 
//         Node* pparentNode = openList.top();  
//         printf("parentNode G H F: %f %f %f;\n", pparentNode->GetGValue(), pparentNode->GetHeuristics(), pparentNode->GetFValue());
//         openList.pop();
//         closedList[GetNodeIndex(pparentNode)] = pparentNode;
        
//         int newX = pparentNode->GetPoseX();
//         int newY = pparentNode->GetPoseY();
//         //  for every successor s' of s such that s' not in CLOSED;  
//         for (int dir=0; dir<NUMOFDIRS; dir++){ // need to calculate time, which increases by +1
//             newX += mdX[dir];
//             newY += mdY[dir];
//             int newIndex = GetIndexFromPose(newX, newY);
//             Node succNode(newX, newY, currTime);

//             // printf("succnode pose: %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY());
//             psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, pgoalNode));
//             // printf("succnode Euclidean: %f;\n", psuccNode->GetHeuristics());
            
//             if (closedList.find(newIndex) == closedList.end()){ // If NOT visited    
//                 // printf("Not Visited.\n");
//                 if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){
//                     // printf("BEFORE psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
                    
//                     psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
//                     psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
//                     psuccNode->SetParent(pparentNode);
//                     // printf("AFTER psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
//                     openList.push(psuccNode);

//                     // printf("succNodeFval: %f;\n", succNode_Fval);
//                     // double fvalue = openList.top()->GetFValue();
//                     // printf("opentop fval: %f;\n", fvalue);
//                     // int size = openList.size();
//                     // printf("openlist size is: %d;\n", size);
//                 }    
//             } else { // If visited
//                 Node* pexistingNode = closedList[newIndex];
//                 // printf("Already visited.\n");
//                 if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
//                     // printf("BEFORE psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
//                     psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
//                     psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
//                     psuccNode->SetParent(pparentNode);
//                     // printf("AFTER psucc G, H, F %f %f %f;\n",psuccNode->GetGValue(), psuccNode->GetHeuristics(), psuccNode->GetFValue());
//                     // closedList[newIndex] = psuccNode;
//                     openList.push(psuccNode);
//                     // double fvalue = openList.top()->GetFValue();
//                     // printf("existing opentop fval: %f;\n", fvalue);
//                     // printf("Pushed to open list.***\n\n");   
//                 }
//             }
//             // printf("succNode F, G, H: %f %f %f;\n", psuccNode->GetFValue(), psuccNode->GetGValue(), psuccNode->GetHeuristics());

//         }

//     } 
//     printf("Size of closed list: %d;\n", closedList.size());
//     printf("Astar Complete\n");
//     return;
// }