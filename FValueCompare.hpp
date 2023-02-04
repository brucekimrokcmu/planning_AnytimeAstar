#pragma once
#include "GraphHelper.hpp"

class FValueCompare{
    public:
        bool operator()(GraphHelper::Node n1, GraphHelper::Node n2);
};
