// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_SEARCHNODE_HPP_
#define WESTBOT_ASTAR_SEARCHNODE_HPP_

#include <QtGlobal>

#include "Export.hpp"

#include "Utils.hpp"

namespace WestBot {
namespace AStar {

class WESTBOT_ASTAR_EXPORT SearchNode
{
public:
    SearchNode();
    ~SearchNode() = default;

    NodeType type;

    uint originX;
    uint originY;
    double h;
    uint g;
    uint expandCost;
    double cost;
    double pathCost;
};

}
}

#endif // WESTBOT_ASTAR_SEARCHNODE_HPP_
