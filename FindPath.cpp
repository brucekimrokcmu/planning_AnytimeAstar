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

std::pair<int, int> FindPath::ExecuteAStar(
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
    }
    if (IsCellValid(goalNode)){
        goalNode.SetGValue(mmap[GetNodeIndex(goalNode)]);
        goalNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
        goalNode.SetFValue(ComputeFValue(goalNode.GetGValue(),goalNode.GetHeuristics(), weight));
    }


    std::vector<std::pair<int, int>> path = AStar(startNode, goalNode, curr_time);
    // For AstarwithDijkstra, I will pass DikstraHeuristics as an argument
    // AStarwithDijkstra()
    std::pair<int, int> nextPose = std::make_pair(path[1].first, path[1].second);


    return nextPose;
}

std::vector<std::pair<int, int>> FindPath::AStar(Node startNode, Node goalNode, int currTime)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;
    double weight = 1.0;    
    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0.0);
    pstartNode->SetHeuristics(ComputeEuclideanHeuristics(startNode, goalNode));
    pstartNode->SetFValue(ComputeFValue(startNode.GetGValue(), startNode.GetHeuristics(), weight));
    openList.push(pstartNode); 
    
    while((closedList.find(GetNodeIndex(goalNode)) == closedList.end()) && (!openList.empty())) {
        Node* pparentNode = openList.top();  
        openList.pop();
        visitedList[GetNodeIndex(pstartNode)] = pstartNode;
        closedList[GetNodeIndex(*pparentNode)] = pparentNode;
        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
        
            Node* psuccNode = new Node(newX, newY, currTime+1); // need to calculate time, which increases by +1    
            if (IsCellValid(psuccNode)) {                            
                psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, &goalNode));                
                if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                    if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
                    Node* pexistingNode = closedList.at(newIndex);
                        if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                            psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                            psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                            psuccNode->SetParent(pparentNode);
                            openList.push(psuccNode);                      
                        }      
                    }                     
                } else { // If NOT visited 
                    visitedList[newIndex]=psuccNode;
                    if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);                     
                    }                     
                }            
            } 
            else {
                // printf("Invalid Cell.\n");
                continue;
            }
        }
        if (closedList.find(GetNodeIndex(goalNode)) != closedList.end()) {
            goalNode.SetParent(pparentNode);

            Node* p = goalNode.GetParent();

            while (p != nullptr) {
                path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                p = p->GetParent();
                
            }
            std::reverse(path.begin(), path.end());
        }    
    }
    // loop through closedlist, openlist and delete all elements    
    for (auto i=closedList.begin(); i != closedList.end();i++) {
        delete i->second;
    }

    while (!openList.empty()){
        delete openList.top();
        openList.pop(); // deallocates memory
    }

    return path;
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
    return fValue;
}

std::unordered_map<int, double> FindPath::Get2DDijkstraHeuristicsTable(Node startNode, Node goalNode, int currTime) 
{
    std::unordered_map<int, double> heuristicsTable;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList;     
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;

    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0);
    pstartNode->SetHeuristics(0);
    pstartNode->SetFValue(ComputeFValue(pstartNode->GetGValue(), pstartNode->GetHeuristics(), 0));
    openList.push(pstartNode); 
    // printf("push startnode to openlist;\n");
    while((!openList.empty())) {
        Node* pparentNode = openList.top();  
        openList.pop();
        visitedList[GetNodeIndex(pstartNode)] = pstartNode;
        closedList[GetNodeIndex(*pparentNode)] = pparentNode;
        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);

            Node* psuccNode = new Node(newX, newY, currTime+1);
            // printf("creating succ node\n");
            if (IsCellValid(psuccNode)) {
                if (visitedList.find(newIndex) != visitedList.end()){ // If visited 
                    if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
                    Node* pexistingNode = closedList.at(newIndex);
                        if(pexistingNode->GetGValue() > pparentNode->GetGValue()+mmap[newIndex]){
                            psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                            psuccNode->SetHeuristics(0);
                            psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(),0));
                            psuccNode->SetParent(pparentNode);
                            openList.push(psuccNode);
                        }      
                    } 
                } else { // If NOT visited 
                    visitedList[newIndex]=psuccNode;
                    if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);              
                        psuccNode->SetHeuristics(0);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(),0));  
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);
                    } 
                }
            } 
            else {
                // printf("Invalid Cell.\n");
                continue;
            }
        }

        if (closedList.find(GetNodeIndex(goalNode)) != closedList.end()) {
            // printf("goal is found");
            goalNode.SetParent(pparentNode);

            Node* p = goalNode.GetParent();
            while (p != nullptr) {
                heuristicsTable[GetNodeIndex(*p)] = p->GetHeuristics();
                p = p->GetParent();   
            }
        }    
    }
    // loop through closedlist, openlist and delete all elements    
    for (auto i=closedList.begin(); i != closedList.end();i++) {
        delete i->second;
    }
    
    while (!openList.empty()){
        openList.pop(); // deallocates memory
    }
    // printf("D* ran;\n");
    return heuristicsTable;
}

// int FindPath::BackwardAStarforDijkstra(Node startNode, Node goalNode, int currTime)
// {
//     std::unordered_map<int, std::vector<int>
//     std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
//     std::unordered_map<int, Node*> closedList;
//     std::unordered_map<int, Node*> visitedList;
//     double weight = 0.0;    
//     Node* pstartNode = new Node(startNode);
//     openList.push(pstartNode); 
    
//     while((closedList.find(GetNodeIndex(goalNode)) == closedList.end()) && (!openList.empty())) {
//         Node* pparentNode = openList.top();  
//         openList.pop();
//         visitedList[GetNodeIndex(pstartNode)] = pstartNode;
//         closedList[GetNodeIndex(*pparentNode)] = pparentNode;
//         for (int dir=0; dir<NUMOFDIRS; dir++){ // need to calculate time, which increases by +1    
//             int newX = pparentNode->GetPoseX() + mdX[dir];
//             int newY = pparentNode->GetPoseY() + mdY[dir];
//             int newIndex = GetIndexFromPose(newX, newY);
        
//             Node* psuccNode = new Node(newX, newY, currTime);
//             if (IsCellValid(psuccNode)) {                            
//                 psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, &goalNode));                
//                 if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
//                     if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
//                     Node* pexistingNode = closedList.at(newIndex);
//                         if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
//                             psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
//                             psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
//                             psuccNode->SetParent(pparentNode);
//                             openList.push(psuccNode);                      
//                         }      
//                     }                     
//                 } else { // If NOT visited 
//                     visitedList[newIndex]=psuccNode;
//                     if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
//                         psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
//                         psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
//                         psuccNode->SetParent(pparentNode);
//                         openList.push(psuccNode);                     
//                     }                     
//                 }            
//             } 
//             else {
//                 // printf("Invalid Cell.\n");
//                 continue;
//             }
//         }
//         if (closedList.find(GetNodeIndex(goalNode)) != closedList.end()) {
//             heuristic = goalNode.GetGValue();
//         }    
//     }
//     // loop through closedlist, openlist and delete all elements    
//     for (auto i=closedList.begin(); i != closedList.end();i++) {
//         delete i->second;
//     }

//     while (!openList.empty()){
//         openList.pop(); // deallocates memory
//     }

//     return heuristic;
// }

