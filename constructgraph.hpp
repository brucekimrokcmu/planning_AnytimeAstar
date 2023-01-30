#ifndef CONSTRUCTGRAPH_H_
#define CONSTRUCTGRAPH_H_
#include <vector>

class ConstructGraph{
    public:
        ConstructGraph(
            double* map, 
            int collision_thresh, 
            int x_size, int y_size, 
            int robotposeX, int robotposeY,
            int target_steps, int curr_time);

        struct state{
            int t;
            int x;
            int y;
            int cost;
        };

        std::vector<int> make_small_graph(state state, //?
            const double* map, const int collision_thresh, const int x_size, const int y_size,
            int robotposeX, int robotposeY, const int target_steps, int curr_time); 





    private:
        double* map;
        int collision_thresh; 
        int x_size, y_size;
        int robotposeX, robotposeY;
        int target_steps, curr_time;


};

#endif