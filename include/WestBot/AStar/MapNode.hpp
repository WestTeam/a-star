// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_MAPNODE_HPP_
#define WESTBOT_ASTAR_MAPNODE_HPP_

#include <QList>
#include <QPair>

#include "Export.hpp"

#include "Utils.hpp"

namespace WestBot {
namespace AStar {

template< typename T >
class WESTBOT_ASTAR_EXPORT MapNode
{
public:
    NodeType type;

    uint expandCost;

    T pixmapItem;
};

template< typename T >
struct WESTBOT_ASTAR_EXPORT RunResources
{
    class MapNode< T >** map;
    QList< QPair< uint, uint> >* path;
    QList< QList< NodeState > >* steps;
    QPair< uint, uint > start;
    QPair< uint, uint > end;
};

}
}

#endif // WESTBOT_ASTAR_MAPNODE_HPP_
