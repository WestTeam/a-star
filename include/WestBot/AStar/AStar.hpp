// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_ASTAR_HPP_
#define WESTBOT_ASTAR_ASTAR_HPP_

#include <cmath>
#include <limits>

#include <QList>
#include <QPair>

#include "Export.hpp"

#include "NodeState.hpp"
#include "SearchNode.hpp"
#include "Utils.hpp"

namespace WestBot {
namespace AStar {

class WESTBOT_ASTAR_EXPORT AStar
{
public:
    AStar();
    AStar( uint width, uint height );
    ~AStar();

    void setMatrix( uint width, uint height );
    void setStart( uint x, uint y );
    void setEnd( uint x, uint y );
    void setWall( uint x, uint y );
    void setWay( uint x, uint y );
    void setHeuristics( AStarHeuristics heuristic );
    void setExpandCost( uint expandCost, uint x, uint y );

    NodeType getNodeType( uint x, uint y ) const;
    QPair< uint, uint > getStart() const;
    QPair< uint, uint > getEnd() const;
    QList< QList< NodeState > > getChanges() const;

    double estimateCost( uint x1, uint y1, uint x2, uint y2 );

    void destroyMatrix();

    QList< QPair< uint, uint > > getPath( bool saveChanges = false );

private:
    void calculateCost( QPair< uint, uint > node);
    void openNode(
        uint x,
        uint y,
        uint originX,
        uint originY,
        QList< NodeState >* changes = NULL );
    void freeNodes();

    void search( bool saveChanges = false );

    void setOriginNode( uint x, uint y, uint originX, uint originY );

    double pathCostToNode( uint x, uint y, uint destX, uint destY );

private:
    SearchNode** _matrix;

    uint _width;
    uint _height;
    QPair< uint, uint > _start;
    QPair< uint, uint> _end;

    QList< QPair< uint, uint > > _path;
    QList< QPair< uint, uint > > _openedNodes;
    QList< QList< NodeState > > _matrixsChanges;

    AStarHeuristics _heuristic;

    bool _ready;
};

}
}

#endif // WESTBOT_ASTAR_ASTAR_HPP_
