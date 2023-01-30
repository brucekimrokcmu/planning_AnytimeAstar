#ifndef COMPUTEPATH_H_
#define COMPUTEPATH_H_

class ComputePath{
    public:
        ComputePath();

        void ComputePath::remove_s_with_smallest_f();
        void ComputePath::insert_s_into_closed();


        double ComputePath::get_gvalue();
        double ComputePath::get_cost();
        double ComputePath::get_heurisitcs();

    private:
        int s;

};

#endif