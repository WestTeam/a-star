// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_PATHNODE_HPP_
#define WESTBOT_ASTAR_PATHNODE_HPP_

#include <QtGlobal>

#include "Export.hpp"

namespace WestBot {
namespace AStar {

class WESTBOT_ASTAR_EXPORT PathNode
{
public:
    PathNode();
    ~PathNode() = default;

    uint x;
    uint y;
};

}
}

#endif // WESTBOT_ASTAR_PATHNODE_HPP_
