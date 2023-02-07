/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

#include "FindPath.hpp"
#include <mex.h>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))


static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{    
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // for now greedily move towards the final target position,
    // but this is where you can put your planner

    //  B: to make goalpose dynamic w.r.t time
    // int goalposeX = (int) target_traj[curr_time-1];
    // int goalposeY = (int) target_traj[curr_time-1+target_steps];

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double eps = 1.0;
    
    //Initialize start/goal node
    Node startNode(robotposeX, robotposeY, curr_time);    
    Node goalNode(goalposeX, goalposeY, curr_time);

    int startX = startNode.GetPoseX(); 
    int startY = startNode.GetPoseY();
    int startT = startNode.GetCurrentTime();
    int goalX = goalNode.GetPoseX();
    int goalY = goalNode.GetPoseY();

    // heuristics #1: Euclidean distance to goalNode
    // heuristics #2: in Backward A*, h would be g*value in the forward 3D A*
    
    if (FindPath::IsCellValid(startNode)) {
        startNode.SetGValue(0.0);
        startNode.SetHeuristics(FindPath::ComputeEuclideanHeuristics(startNode, goalNode));
        startNode.SetFValue(FindPath::ComputeFValue(startNode, eps));
    }

    if (FindPath::IsCellValid(goalNode)){
        startNode.SetGValue(FindPath::GetCellCost(goalNode));
        startNode.SetHeuristics(FindPath::ComputeEuclideanHeuristics(goalNode, goalNode));
        startNode.SetFValue(FindPath::ComputeFValue(goalNode, eps));
    } // else what? 



    std::vector<Node> smallGraph;
    smallGraph.push_back(startNode);


    /////////////////////LOOP STARTS HERE///////////////////////////////////////////
    // loop until when? 
    
    // add surrounding nodes
    for (int dir=0; dir<NUMOFDIRS; dir++) {

        int newX = startX + dX[dir];
        int newY = startY + dX[dir];
        // Get a new node here 
        Node newNode(newX, newY, curr_time, map, x_size, y_size);

        // check collision, boundary
        if (newX >= 1 && newX <= x_size && newY >= 1 && newY <= y_size) {
            if (((int)map[GETMAPINDEX(newX,newY,x_size,y_size)] >= 0) && 
                ((int)map[GETMAPINDEX(newX,newY,x_size,y_size)] < collision_thresh)) {
                
                newNode.SetBoolClosed(false);
                newNode.SetBoolExpanded(false);

                double gValue = (double)map[GETMAPINDEX(newX,newY,x_size,y_size)];
                newNode.SetGValue(gValue);

                // heuristics #1: Euclidean distance to goalNode
                double heuristic = (double)std::sqrt(((goalX - newX)*(goalX-newX) + (goalY - newY)*(goalY-newY)));
                newNode.SetHeuristics(heuristic);

                smallGraph.push_back(newNode);
            }
        } // else what? 
    }

    // A*
    // std::priority_queue<Node, std::vector<Node>, FValueCompare> openList; 
    std::priority_queue<Node*, std::vector<Node*>, FValueCompare> openList; 

    goalNode.SetBoolExpanded(false);
    openList.push(&startNode); // OPEN = {s_start}; 


    // while(s_goal is not expanded && OPEN!=0){ //i.e. s_goal(=goalpose) is not in the CLOSED list
    while((!goalNode.GetBoolExpanded()) && (!openList.empty())) {
    // remove s with the smallest [f(s)=g(s)+h(s)] from OPEN;
    //  insert s into CLOSED;
        Node* ref = openList.top(); // this is copying. might want to try reference
        ref->SetBoolClosed(true);
        // openList.pop();

    //  for every successor s' of s such that s' not in CLOSED;
        for (int i=0; i<smallGraph.size(); i++){
            Node n_ref = smallGraph[i]

        }
    //     if g(s') > g(s) + c(s, s')


//             g(s') = g(s) + c(s, s')
//             insert s' into OPEN;

    } 

    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;



    ////////////////////////////////LOOP ENDS HERE //////////////////////////////

   // 3. publish action(solution)
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}



// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];


    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}