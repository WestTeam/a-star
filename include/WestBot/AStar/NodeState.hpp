// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_NODESTATE_HPP_
#define WESTBOT_ASTAR_NODESTATE_HPP_

#include <QtGlobal>

#include "Export.hpp"

#include "Utils.hpp"

namespace WestBot {
namespace AStar {

class WESTBOT_ASTAR_EXPORT NodeState
{
public:
    NodeState();
    NodeState( uint x, uint y, NodeType state );
    ~NodeState() = default;

    uint x;
    uint y;

    NodeType state;

    AncestorDirection ancestor;
};

}
}

#endif // WESTBOT_ASTAR_NODESTATE_HPP_
