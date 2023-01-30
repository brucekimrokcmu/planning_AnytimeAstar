#include "constructgraph.hpp"

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
#define NUMOFDIRS 8

ConstructGraph::ConstructGraph(
    double* map, 
    int collision_thresh, 
    int x_size, int y_size, 
    int robotposeX, int robotposeY,
    int target_steps, int curr_time)
    : map(map), 
    collision_thresh(collision_thresh), 
    x_size(x_size), y_size(y_size), 
    robotposeX(robotposeX), robotposeY(robotposeY),
    target_steps(target_steps), curr_time(curr_time)
{};




// heuristics as an argument needed
std::vector<ConstructGraph::Node> ConstructGraph::make_small_graph(Node n,
    const double* map, const int collision_thresh, const int x_size, const int y_size,
    int robotposeX, int robotposeY, const int target_steps, int curr_time)
{
    std::vector<ConstructGraph::Node> small_graph; // return an array of sturcture 

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    for (int dir=0; dir<8; dir++){
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) 
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))
            {
                // store state as a structure  


            }
        }
    }


    return small_graph;

}