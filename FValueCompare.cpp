#include "FValueCompare.hpp"

bool FValueCompare::operator()(Node node1, Node node2)
{
    int fValueNode1 =node1.GetGValue()+node1.GetHeuristics();
    int fValueNode2 =node2.GetGValue()+node2.GetHeuristics();

    if ((fValueNode1) <= (fValueNode2)) {
        return true;
    } else {
        return false;
    }
}
