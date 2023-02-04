#include "FValueCompare.hpp"

bool FValueCompare::operator()(GraphHelper::Node n1, GraphHelper::Node n2)
{
    int fValueN1 =n1.GetGValue()+n1.GetHeuristics();
    int fValueN2 =n2.GetGValue()+n2.GetHeuristics();

    if ((fValueN1) <= (fValueN2)) {
        return true;
    } else {
        return false;
    }
}
