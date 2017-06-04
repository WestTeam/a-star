// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_UTILS_HPP_
#define WESTBOT_ASTAR_UTILS_HPP_

#include "Export.hpp"

namespace WestBot {
namespace AStar {

enum class ToSetState
{
    TOSETWALL,
    TOSETWAY,
    TOSETDIRT,
    TOSETSTART,
    TOSETEND
};

enum class NodeType
{
    NONE = 0,
    WAYNODE,
    WALLNODE,
    OPENEDNODE,
    VISITEDNODE
};

enum class AStarHeuristics
{
    none,
    euclidean,
    manhattan,
    diagonal,
    newH
};

enum class AncestorDirection
{
    SELF,
    UPLEFT,
    UP,
    UPRIGHT,
    LEFT,
    RIGHT,
    DOWNLEFT,
    DOWN,
    DOWNRIGHT
};

}
}

#endif // WESTBOT_ASTAR_UTILS_HPP_
