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
//   mDHeuristics = ComputeBackwardDijkstra();   
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
    Node goalNode(targetposeX, targetposeY, curr_time); 

    // Compute heuristics
    if (!IsCellValid(startNode)) {
        printf("Invalid StartNode!\n");
    }
    if (IsCellValid(goalNode)){
        goalNode.SetGValue(mmap[GetNodeIndex(goalNode)]);
        goalNode.SetHeuristics(ComputeEuclideanHeuristics(goalNode, goalNode));
        goalNode.SetFValue(ComputeFValue(goalNode.GetGValue(),goalNode.GetHeuristics(), weight));
    }

    std::vector<std::pair<int, int>> path = AStar(startNode, goalNode, curr_time);
    std::pair<int, int> nextPose = std::make_pair(path[1].first, path[1].second);

    return nextPose;
}



std::vector<std::pair<int, int>> FindPath::ExecuteMultigoalAStar(
                     int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr
                     )
{
    Node startNode(robotposeX, robotposeY, curr_time); 
    if (!IsCellValid(startNode)) {
        printf("NODE INVALID");
    }
    int targetTime = 400;
    int dT = 200;
    
    printf("get path and return the index one\n");
    std::vector<std::pair<int,int>> path = MultigoalAStar(startNode, curr_time, targetTime);
    
    
    // printf("exits multigoalstar function\n");

    // printf("retrieves next pose from the path.\n");
    // printf("next pose x y: %d %d\n", nextPose.first, nextPose.second);
    return path;
}



std::vector<std::pair<int, int>> FindPath::ExecuteMultigoalAstarWithDijkstraHeuristics(
                     int robotposeX, 
                     int robotposeY,
                     int targetposeX,
                     int targetposeY,
                     int curr_time,
                     double* action_ptr,
                     std::unordered_map<int, double>* pg_DHueristics
                    )
{

    // using the Dijkstra as heuristics, run Astar and return path

    // replan when targetTime < currTime
    
    Node startNode(robotposeX, robotposeY, curr_time); 
    // printf("get path and return the index one\n");
    std::vector<std::pair<int,int>> path = AStarwithMultiBackwardDijkstra(startNode, curr_time, pg_DHueristics);
    
    // printf("exits multigoalstar function\n");

    // printf("retrieves path.\n");
    // printf("next pose x y: %d %d\n", nextPose.first, nextPose.second);
    // for (int i=0; i<path.size(); i++){
    //         printf("path[i] x y :%d %d %d\n", i, path[i].first, path[i].second);
    // }
    return path;

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


        visitedList[GetNodeIndex(pparentNode)] = pparentNode;
        closedList[GetNodeIndex(pparentNode)] = pparentNode;     

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
            // When checking the goal, I need to check the time

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

    if(openList.empty()){
        printf("openlist is already empty.\n");
    } else {
        while (!openList.empty()){
        printf("deallocating memory from openlist\n");
        delete openList.top();
        openList.pop(); // deallocates memory
    }

    }

    
    return path;
}

std::vector<std::pair<int, int>> FindPath::MultigoalAStar(Node startNode, int currTime, int targetTime)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> goalList; // a separate goallist     
    std::unordered_map<int, Node*> visitedList;
    //create a goallist
    for (int t=0; t<mtargetSteps;t++){
        Node* pgoalNode = new Node((int)mtargetTrajectory[t], (int)mtargetTrajectory[t+mtargetSteps], t);
        goalList[GetNodeIndex(pgoalNode)] = pgoalNode;    
    }
    
    double weight = 200.0;    
    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0.0);
    pstartNode->SetHeuristics(ComputeEuclideanHeuristics(pstartNode, goalList[GetIndexFromPose((int)mtargetTrajectory[0], (int)mtargetTrajectory[mtargetSteps])]));
    pstartNode->SetFValue(ComputeFValue(pstartNode->GetGValue(), pstartNode->GetHeuristics(), weight));
    openList.push(pstartNode); 
    // printf("initialized pstartNode and pushed into openlist.\n");
    // check time -> extract goal from the trajectory
    // I can also check the elapsed time running AStar and add that to goal time.
    
    while((!openList.empty())) {
        if(IsCellValid(openList.top())){
            Node* pparentNode = openList.top();  
            openList.pop();
            
            // printf("parent node x y t: %d %d %d\n", pparentNode->GetPoseX(), pparentNode->GetPoseY(), pparentNode->GetCurrentTime());
            visitedList[GetNodeIndex(pparentNode)] = pparentNode;
            closedList[GetNodeIndex(pparentNode)] = pparentNode;
            // check if parentnode == every single goal nodes along with time 
            // pparentnode.time <= goal time 
            currTime = pparentNode->GetCurrentTime();
            // targetTime >= currTime
            Node* ptargetGoalNode = goalList[GetIndexFromPose((int)mtargetTrajectory[targetTime], (int)mtargetTrajectory[targetTime+mtargetSteps])];
            // printf("goal   node x y t: %d %d %d\n", ptargetGoalNode->GetPoseX(), ptargetGoalNode->GetPoseY(), ptargetGoalNode->GetCurrentTime());
            // printf("popped openlist\n");
            // printf("ptargetgoal created.\n");
            
            if (GetNodeIndex(pparentNode) == GetNodeIndex(ptargetGoalNode)) {
                printf("parent meets goal\n");
                ptargetGoalNode->SetParent(pparentNode);
                
                Node* p = ptargetGoalNode->GetParent();
                while (p != nullptr) {
                    path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                    p = p->GetParent();    
                }
                std::reverse(path.begin(), path.end());
                
                // for (int i=0; i<3; i++){
                //     printf("next pose x y: %d %d \n", path[i].first, path[i].second);
                // }

                while(!openList.empty()){
                    delete openList.top();
                    // printf("deallocating memory from openlist\n");
                    openList.pop();
                }
                break;
            }    

            for (int dir=0; dir<NUMOFDIRS; dir++){ 
                int newX = pparentNode->GetPoseX() + mdX[dir];
                int newY = pparentNode->GetPoseY() + mdY[dir];
                int newIndex = GetIndexFromPose(newX, newY);
                // printf("num of succ: %d\n", dir);
                if (newX >= 1 && newX <= mxSize && newY >= 1 && newY <= mySize) {    
                    if (((int)mmap[newIndex] >= 0) && ((int)mmap[newIndex] < mcollisionThresh)) {
                        Node* psuccNode = new Node(newX, newY, currTime+1);   
                        // printf("psucc node x y t: %d %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY(), psuccNode->GetCurrentTime());
                        if (IsCellValid(psuccNode)) {                            
                            psuccNode->SetHeuristics(ComputeEuclideanHeuristics(psuccNode, ptargetGoalNode));                
                            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                                // printf("Visited.\n");
                                if (closedList.find(newIndex) != closedList.end()){  //If inside the closde list
                                Node* pexistingNode = closedList.at(newIndex);
                                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                                        psuccNode->SetParent(pparentNode);
                                        openList.push(psuccNode);   
                                        // printf("updated closed list and pushed to openlist.\n\n");                   
                                        
                                        
                                    } 
                                     
                                } else {
                                    if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
                                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                                        psuccNode->SetParent(pparentNode);
                                        openList.push(psuccNode);     
                                    
                                    
                                    // printf("pushed to open list.\n\n");                
                                    }
                                }                    
                            } else { // If NOT visited 
                                visitedList[newIndex]=psuccNode;
                                // printf("Not visited\n");
                                if (psuccNode->GetGValue()> pparentNode->GetGValue()+mmap[newIndex]){                
                                    psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                                    psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                                    psuccNode->SetParent(pparentNode);
                                    openList.push(psuccNode);     
                                    
                                    
                                    // printf("pushed to open list.\n\n");                
                                }                     
                            }            
                        } 
                        else {
                            printf("Invalid succNode.\n");
                            continue;
                        }
                    } else {
                        continue;
                    }
                } else {
                    continue;
                }
            }
            
        }
    }

    // printf("exists while loop\n");
    // loop through closedlist, openlist, goallist and delete all elements    

    for (auto i=goalList.begin(); i != goalList.end();i++){
        delete i->second;
    }
    // printf("goal list is deleted\n");

   for (auto i=closedList.begin(); i != closedList.end();i++){
        delete i->second;
    }
    // printf("closed list is deleted\n");
    // for (auto i=visitedList.begin(); i != visitedList.end();i++) {
    //     delete i->second;
    // }
    // printf("visited list is deleted\n");


    return path;
}

std::vector<std::pair<int, int>> FindPath::AStarwithMultiBackwardDijkstra(Node startNode, int currTime, std::unordered_map<int, double>* pg_DHueristics)
{
    std::vector<std::pair<int, int>> path;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> goalList;     
    std::unordered_map<int, Node*> visitedList;
    //create a goallist
    for (int t=0; t<mtargetSteps;t++){
        Node* pgoalNode = new Node((int)mtargetTrajectory[t], (int)mtargetTrajectory[t+mtargetSteps], t);
        goalList[GetNodeIndex(pgoalNode)] = pgoalNode;    
    
    }
    // printf("goal list saved\n");
    
    double weight = 50.0;    
    Node* pstartNode = new Node(startNode);
    pstartNode->SetGValue(0.0);
    // printf("start setting heuristics of the startnode\n");
    pstartNode->SetHeuristics((*pg_DHueristics)[GetNodeIndex(pstartNode)]);
    // printf("I got it! it's %f\n", pstartNode->GetHeuristics());
    pstartNode->SetFValue(ComputeFValue(pstartNode->GetGValue(), pstartNode->GetHeuristics(), weight));
    openList.push(pstartNode); 
    // printf("initialized pstartNode and pushed into openlist.\n");
    // check time -> extract goal from the trajectory
    // I can also check the elapsed time running AStar and add that to goal time.
    
    int targetTime = 100;
    Node* ptargetGoalNode = goalList[GetIndexFromPose((int)mtargetTrajectory[targetTime], (int)mtargetTrajectory[targetTime+mtargetSteps])];
    // printf("ptargetgoal created.\n");
    

    // printf("starting while loop!\n");
    while((!openList.empty())) {
        Node* pparentNode = openList.top();  
        if(!IsCellValid(openList.top()))
            continue;
        // printf("parent node x y t: %d %d %d\n", pparentNode->GetPoseX(), pparentNode->GetPoseY(), pparentNode->GetCurrentTime());
        openList.pop();
        
        visitedList[GetNodeIndex(pparentNode)] = pparentNode;
        closedList[GetNodeIndex(pparentNode)] = pparentNode;

        // check if parentnode == every single goal nodes along with time 
        // pparentnode.time <= goal time 
        currTime = pparentNode->GetCurrentTime();
        
        
        
        if (GetNodeIndex(pparentNode) == GetNodeIndex(ptargetGoalNode)) {
            // printf("parent meets goal\n");
            // printf("goal   node x y t: %d %d %d\n", ptargetGoalNode->GetPoseX(), ptargetGoalNode->GetPoseY(), ptargetGoalNode->GetCurrentTime());
            ptargetGoalNode->SetParent(pparentNode);
            Node* p = ptargetGoalNode->GetParent();
            while (p != nullptr) {
                path.push_back(std::make_pair(p->GetPoseX(), p->GetPoseY()));
                // printf("reversed x y : %d %d \n", p->GetPoseX(), p->GetPoseY());
                p = p->GetParent();    
            }
            std::reverse(path.begin(), path.end());
            
            // for (int i=0; i<3; i++){
            //     printf("next pose x y: %d %d \n", path[i].first, path[i].second);
            // }

            while(!openList.empty()){
                delete openList.top();
                openList.pop();
            }
            // printf("exist while loop because parent is found;\n");
            break;// printf("Not visited\n");
        }    

        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);
            
            if (!inBounds(newX, newY))
                continue;

            if (!isCollisionFree(newIndex))
                continue;
            
            Node* psuccNode = new Node(newX, newY, currTime+1);   
            // printf("psucc node x y t: %d %d %d;\n", psuccNode->GetPoseX(), psuccNode->GetPoseY(), psuccNode->GetCurrentTime());
                           
            psuccNode->SetHeuristics((*pg_DHueristics)[GetNodeIndex(psuccNode)]);                
            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                // printf("Visited.\n");
                if (closedList.find(newIndex) == closedList.end()){  //If inside the closde list
                    Node* pexistingNode = visitedList.at(newIndex);
                    
                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]+weight*(psuccNode->GetHeuristics())){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);  
                        visitedList[newIndex] = psuccNode; 
                        // printf("updated visited list and pushed to openlist.\n\n");                  
                    } 
                        
                } 
            } else { // If NOT visited 
                // printf("Not visited\n");
                visitedList[newIndex]=psuccNode;                            
                psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), psuccNode->GetHeuristics(), weight));                
                psuccNode->SetParent(pparentNode);
                openList.push(psuccNode);                                         
                // printf("pushed to open list.\n\n");                                                     
            }                        
        }                     
    }

    // printf("exists while loop because open list is empty\n");
    // loop through closedlist, openlist, goallist and delete all elements    

    for (auto i=goalList.begin(); i != goalList.end();i++){
        delete i->second;
    }
    // printf("goal list is deleted\n");

   for (auto i=closedList.begin(); i != closedList.end();i++){
        delete i->second;
    }
    // printf("closed list is deleted\n");
    // for (auto i=visitedList.begin(); i != visitedList.end();i++) {
    //     delete i->second;
    // }
    // printf("visited list is deleted\n");


    return path;
}

/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/
/*######################################################################################################################################*/

bool inline inBounds(int x, int y, int maxX, int maxY)
{
    return x >= 1 && x <= maxX && y >= 1 && y <= maxY;
}

bool inline isCollisionFree(int idx, int threshold, double* map)
{
    return map[idx] >= 0 && map[idx] < threshold;
}

std::unordered_map<int, double> FindPath::ComputeBackwardDijkstra() 
{
    printf("D* run\n");
    std::unordered_map<int, double> heuristicsTable;
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList;     
    std::unordered_map<int, Node*> closedList;
    std::unordered_map<int, Node*> visitedList;

    for (int t=0; t<mtargetSteps;t++){
        Node* pgoalNode = new Node((int)mtargetTrajectory[t], (int)mtargetTrajectory[t+mtargetSteps], t);
        pgoalNode->SetCurrentTime(t);
        pgoalNode->SetGValue(0.0);
        pgoalNode->SetFValue(pgoalNode->GetGValue());
        openList.push(pgoalNode);

    }

    double weight = 0.0;
    
    while((!openList.empty())) {
        // printf("open list size: %d\n", openList.size());
        // printf("closed list size: %d\n", closedList.size());

        Node* pparentNode = openList.top();

        if(!IsCellValid(openList.top()))
            continue;

        openList.pop();

        visitedList[GetNodeIndex(pparentNode)] = pparentNode;
        closedList[GetNodeIndex(pparentNode)] = pparentNode;

        for (int dir=0; dir<NUMOFDIRS; dir++){ 
            // Calculate new position of node
            int newX = pparentNode->GetPoseX() + mdX[dir];
            int newY = pparentNode->GetPoseY() + mdY[dir];
            int newIndex = GetIndexFromPose(newX, newY);

            if (!inBounds(newX, newY))
                continue;

            if (!isCollisionFree(newIndex))
                continue;
            
            Node* psuccNode = new Node(newX, newY, 0);
                                
            psuccNode->SetHeuristics(0.0);                
            if (visitedList.find(newIndex) != visitedList.end()){ // If visited                     
                // printf("Visited.\n");

                if (closedList.find(newIndex) == closedList.end()){  //If not inside the closde list
                    Node* pexistingNode = visitedList.at(newIndex);
                    
                    if(pexistingNode->GetFValue() > pparentNode->GetGValue()+mmap[newIndex]){
                        psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                        psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), 0, 0));
                        psuccNode->SetParent(pparentNode);
                        openList.push(psuccNode);
                        visitedList[newIndex] = psuccNode;
                        // printf("updated closed list and pushed to openlist.\n\n");                   
                    } 
                }                    
            } else { // If NOT visited 
            
                visitedList[newIndex]=psuccNode;
                // printf("Not visited\n");
                // Always add the node to the open list since this is the first time seeing it          
                psuccNode->SetGValue(pparentNode->GetGValue()+mmap[newIndex]);
                psuccNode->SetFValue(ComputeFValue(psuccNode->GetGValue(), 0, 0));                
                psuccNode->SetParent(pparentNode);
                openList.push(psuccNode);   
            }
        }            
    }

    // printf("D* exits while loop\n");
    // loop through closedlist, openlist and delete all elements    
    for (auto i=closedList.begin(); i != closedList.end();i++) {
        heuristicsTable[GetNodeIndex(i->second)] = (i->second)->GetGValue();
        // printf("stores heuristics table from closed list\n");
        delete i->second;
    }
    
    while (!openList.empty()){
        heuristicsTable[GetNodeIndex(openList.top())] = openList.top()->GetGValue();
        // printf("stores heuristics table from open list\n");
        delete openList.top();
        openList.pop(); // deallocates memory
    }
    // printf("D* ran;\n");
    return heuristicsTable;
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

bool inline FindPath::inBounds(int x, int y)
{
    return x >= 1 && x <= mxSize && y >= 1 && y <= mySize;
}

bool inline FindPath::isCollisionFree(int idx)
{
    return mmap[idx] >= 0 && mmap[idx] < mcollisionThresh;
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
