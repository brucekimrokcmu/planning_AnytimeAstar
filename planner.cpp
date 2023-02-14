/*=================================================================
 * planner.cpp
   prhs contains input parameters (4):
   1st is matrix with all the obstacles
   2nd is a row vector <x,y> for the robot position
   3rd is a matrix with the target trajectory
   4th is an integer C, the collision threshold for the map
   plhs should contain output parameters (1):
   1st is a row vector <dx,dy> which corresponds to the action that the robot should make
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
static std::vector<std::pair<int, int>> g_path;
static bool init = false;
static int iter=2;

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

    // targetposeX = (int) target_traj[target_steps-1];
    // targetposeY = (int) target_traj[target_steps-1+target_steps];
    printf("robot  x y t:%d %d %d;\n", robotposeX, robotposeY, curr_time);
    printf("target x y t:%d %d %d;\n", targetposeX, targetposeY, curr_time);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    // printf("curr time: %d;\n", curr_time);

    std::unique_ptr<FindPath> pathPlanner(new FindPath(map, collision_thresh, x_size, y_size, target_steps, target_traj));

    if(!init) {
        std::vector<std::pair<int, int>> path = pathPlanner->ExecuteMultigoalAStar(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);        
        g_path = path;    
        robotposeX = g_path[1].first;
        robotposeY = g_path[1].second;
        // for (int i=0; i<g_path.size(); i++){
        //     printf("path[i] x y :%d %d %d\n", i, g_path[i].first, g_path[i].second);
        // }
        init = true;
    } else if (iter<g_path.size()){
        printf("init is now false\n");
        
        robotposeX = g_path[iter].first;
        robotposeY = g_path[iter].second;
        printf("next robot pose x y: %d %d\n", robotposeX, robotposeY);
        iter++;
        
    }
    
    
    // std::pair<int, int> nextPose = pathPlanner->ExecuteAStar(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);
    // std::pair<int, int> nextPose = pathPlanner->ExecuteAStar2DDijkstra(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);

    // FindPath pathPlanner(map, collision_thresh, x_size, y_size, target_steps, target_traj);
    // std::pair<int, int> nextPose = pathPlanner.Execute(robotposeX, robotposeY, targetposeX, targetposeY, curr_time, action_ptr);
    printf("returns the next pose\n");
    

    // robotposeX = nextPose.first;
    // robotposeY = nextPose.second;
    
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    return;
    /* Only for test
    FindPath planner;
    static bool init = false;
    if (!init) {
    planner = FindPath(<CONSTRUCTOR>);
    init = true;
    }
    */

   /* 
    Singleton
    getSingletonFindpath()->execute()
    
    class GetSing....() {
    
    return static .....'
    }
   */

}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray*prhs[])
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
    if(targettraj_M < 1 || targettraj_N != 2){
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





