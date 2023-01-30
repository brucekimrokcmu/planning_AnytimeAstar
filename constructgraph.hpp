#ifndef CONSTRUCTGRAPH_H_
#define CONSTRUCTGRAPH_H_
#include <vector>

// #define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))


class ConstructGraph{
    public:
        ConstructGraph(
            double* map, 
            int collision_thresh, 
            int x_size, int y_size, 
            int robotposeX, int robotposeY,
            int target_steps, int curr_time);

        class Node{
            public:
                Node(double* map, int robotposeX, int robotposeY, int curr_time)
                    : t(curr_time), x(robotposeX), y(robotposeY)
                    {};
                
                int Node::get_cell_cost(double* map, int x_size, int x, int y)
                {
                    return (int)map[(y-1)*x_size + (x-1)]; //?
                } 
                bool Node::get_flag()
                {
                    return flag;
                } 
                bool Node::set_flag(bool val);
                {   
                    flag = val;
                    return flag;
                }

                int t;
                int x;
                int y;
                int cell_cost;
                bool flag; //open:1, closed:0
        };

        std::vector<ConstructGraph::Node> make_small_graph(Node n,
            const int collision_thresh, const int x_size, const int y_size,
            const int target_steps); 

    private:
        double* map;
        int collision_thresh; 
        int x_size, y_size;
        int robotposeX, robotposeY;
        int target_steps, curr_time;


};

#endif