// Copyright (c) 2016-2017 All Rights Reserved WestBot

#include <WestBot/AStar/AStar.hpp>

using namespace WestBot::AStar;

AStar::AStar()
{
    _matrix = NULL;
    _width = 0;
    _height = 0;
    _heuristic = AStarHeuristics::euclidean;
    _start.first = 0;
    _start.second = 0;
    _end.first = 0;
    _end.second = 0;
    _path.clear();
    _openedNodes.clear();
    _matrixsChanges.clear();
    _ready = false;
}

AStar::AStar( uint width, uint height )
{
    _matrix = new SearchNode*[ height ];

    for( uint i = 0; i < height; ++i )
    {
        _matrix[ i ] = new SearchNode[ width ];
    }

    _width = width;
    _height = height;
    _start.first = 0;
    _start.second = 0;
    _end.first = 0;
    _end.second = 0;
    _path.clear();
    _openedNodes.clear();
    _matrixsChanges.clear();
    _ready = true;
}

AStar::~AStar()
{
    destroyMatrix();

    _path.clear();
    _openedNodes.clear();
}

void AStar::calculateCost( QPair< uint, uint > node )
{
    double x;
    double y;
    double destX;
    double destY;
    double h;
    double cost;
    double euc;
    double pathCost;

    uint g;
    uint originX;
    uint originY;

    x = ( double ) node.first;
    y = ( double ) node.second;
    destX = ( double ) _end.first;
    destY = ( double ) _end.second;

    originX = _matrix[ node.first ][ node.second ].originX;
    originY = _matrix[ node.first ][ node.second ].originY;

    g = _matrix[ originX ][ originY ].g +
        _matrix[ node.first ][ node.second ].expandCost;

    euc = std::sqrt( std::pow( x - destX, 2.0 ) + std::pow( y - destY, 2.0 ) );

    if( _heuristic == AStarHeuristics::none )
    {
        h = 0;
    }
    else if( _heuristic == AStarHeuristics::euclidean )
    {
        h = euc;
    }
    else if( _heuristic == AStarHeuristics::manhattan )
    {
        h = std::abs(x - destX) + abs(y - destY);
    }
    else if( _heuristic == AStarHeuristics::diagonal )
    {
        h = std::max(abs(x - destX), abs(y - destY));
    }
    else if( _heuristic == AStarHeuristics::newH )
    {
        h = std::ceil(euc) - euc;
    }

    cost = h + ( double ) g;

    pathCost = _matrix[ originX ][ originY ].pathCost + cost;

    _matrix[ node.first ][ node.second ].h = h;
    _matrix[ node.first ][ node.second ].g = g;
    _matrix[ node.first ][ node.second ].cost = cost;
    _matrix[ node.first ][ node.second ].pathCost = pathCost;
}

void AStar::openNode(
    uint x,
    uint y,
    uint originX,
    uint originY,
    QList< NodeState >* changes )
{
    NodeState nodeState;
    double orignalPathCost;
    double newPathCost;
    uint predX;
    uint predY;

    if( x >= _width || y >= _height )
    {
        return;
    }

    if( originX >= _width || originY >= _height )
    {
        return;
    }

    if( _matrix[ x ][ y ].type == NodeType::WALLNODE ||
        _matrix[ x ][ y ].type == NodeType::VISITEDNODE )
    {
        return;

    }
    else if( _matrix[ x ][ y ].type == NodeType::WAYNODE )
    {
        _matrix[ x ][ y ].type = NodeType::OPENEDNODE;
        _openedNodes.push_back( QPair< uint, uint >( x, y ) );
    }
    else if( _matrix[ x ][ y ].cost < estimateCost( originX, originY, x, y ) )
    {
        return;
    }
    else if( _matrix[ x ][ y ].expandCost != 1 )
    {
        orignalPathCost = pathCostToNode(
            _matrix[ x ][ y ].originX,
            _matrix[ x ][ y ].originY,
            x,
            y );
        newPathCost = pathCostToNode( originX, originY, x, y );

        predX = _matrix[ x ][ y ].originX;
        predY = _matrix[ x ][ y ].originY;

        if( orignalPathCost < newPathCost )
        {
            return;
        }
    }

    setOriginNode( x, y, originX, originY );
    calculateCost( QPair< uint, uint >( x, y ) );

    if( changes != NULL )
    {
        nodeState.x = x;
        nodeState.y = y;
        nodeState.state = NodeType::OPENEDNODE;

        originX -= x;
        originY -= y;

        if( originX == -1 )
        {
            if( originY == -1 )
            {
                nodeState.ancestor = AncestorDirection::UPLEFT;
            }
            else if( originY == 0 )
            {
                nodeState.ancestor = AncestorDirection::LEFT;
            }
            else
            {
                nodeState.ancestor = AncestorDirection::DOWNLEFT;
            }
        }
        else if( originX == 1 )
        {
            if( originY == -1 )
            {
                nodeState.ancestor = AncestorDirection::UPRIGHT;
            }
            else if( originY == 0 )
            {
                nodeState.ancestor = AncestorDirection::RIGHT;
            }
            else
            {
                nodeState.ancestor = AncestorDirection::DOWNRIGHT;

            }
        }
        else
        {
            if( originY == -1 )
            {
                nodeState.ancestor = AncestorDirection::UP;
            }
            else
            {
                nodeState.ancestor = AncestorDirection::DOWN;
            }
        }

        changes->push_back( nodeState );
    }
}

void AStar::freeNodes()
{
    if( _ready )
    {
        return;
    }

    if( _matrix == NULL )
    {
        return;
    }

    for( uint i = 0; i < _width;  ++i )
    {
        for( uint j = 0; j < _height; ++j )
        {
            if( _matrix[ i ][ j ].type != NodeType::WALLNODE )
            {
                _matrix[ i ][ j ].type = NodeType::WAYNODE;
            }
        }
    }

    _ready = true;
}

void AStar::setOriginNode( uint x, uint y, uint originX, uint originY )
{
    if( x >= _width || y >= _height )
    {
        return;
    }

    if( originX >= _width || originY >= _height )
    {
        return;
    }

    _matrix[ x ][ y ].originX = originX;
    _matrix[ x ][ y ].originY = originY;
}

double AStar::pathCostToNode( uint x, uint y, uint destX, uint destY )
{
    double euc, h;
    double cost = 0;
    uint originX;
    uint originY;

    originX = _matrix[ x ][ y ].originX;
    originY = _matrix[ x ][ y ].originY;

    while( ! ( originX == x && originY == y ) )
    {
        euc = std::sqrt(
            std::pow( ( double ) x - ( double ) destX, 2.0 ) +
            std::pow( ( double ) y - ( double ) destY, 2.0 ) );

        if( _heuristic == AStarHeuristics::none )
        {
            h = 0;
        }
        else if( _heuristic == AStarHeuristics::euclidean )
        {
            h = euc;
        }
        else if( _heuristic == AStarHeuristics::manhattan )
        {
            h = std::abs( ( double ) x - ( double ) destX ) +
                std::abs( ( double ) y - ( double ) destY );
        }
        else if( _heuristic == AStarHeuristics::diagonal )
        {
            h = std::max(
                std::abs( ( double ) x - ( double ) destX ),
                std::abs( ( double ) y - ( double ) destY ) );
        }
        else if( _heuristic == AStarHeuristics::newH )
        {
            h = std::ceil( euc ) - euc;
        }

        cost += h + _matrix[ x ][ y ].expandCost;

        x = originX;
        y = originY;

        originX = _matrix[ x ][ y ].originX;
        originY = _matrix[ x ][ y ].originY;
    }

    return cost;
}

void AStar::search( bool saveChanges )
{
    uint x;
    uint y;
    double minCost;
    QPair< uint, uint > node;
    QList< QPair< uint, uint > >::iterator elem, it;
    QList< NodeState > *newMatrixState;
    NodeState nodeState;

    if( _matrix == NULL )
    {
        return;
    }

    if( _start.first > _width || _start.second > _height )
    {
        return;
    }

    if( _end.first > _width || _end.second > _height )
    {
        return;
    }

    freeNodes();
    _path.clear();
    _matrixsChanges.clear();

    if( _start == _end )
    {
        _path.push_front( _start );
        return;
    }

    _ready = false;

    if( saveChanges )
    {
        newMatrixState = new QList< NodeState >();
    }
    else
    {
        newMatrixState = NULL;
    }

    _openedNodes.clear();
    _openedNodes.push_front( _start );

    x = _start.first;
    y = _start.second;
    setOriginNode( x, y, x, y );
    _matrix[ x ][ y ].cost = 0;
    _matrix[ x ][ y ].pathCost = 0;
    _matrix[ x ][ y ].type = NodeType::OPENEDNODE;
    _matrix[ x ][ y ].g = 0;

    while( _matrix[ _end.first ][ _end.second ].type != NodeType::OPENEDNODE )
    {
        minCost = std::numeric_limits< double >::max();

        if( newMatrixState != NULL )
        {
            newMatrixState->clear();
        }

        if( _openedNodes.size() == 0 )
        {
            return;
        }

        for( auto it = _openedNodes.begin(); it != _openedNodes.end(); ++it )
        {
            x = it->first;
            y = it->second;

            if( _matrix[ x ][ y ].cost < minCost )
            {
                minCost = _matrix[ x ][ y ].cost;
                elem = it;
            }
        }

        x = elem->first;
        y = elem->second;

        _matrix[ x ][ y ].type = NodeType::VISITEDNODE;

        if( newMatrixState != NULL )
        {
            nodeState.x = x;
            nodeState.y = y;
            nodeState.state = NodeType::VISITEDNODE;

            if( _matrix[ x ][ y ].originX == -1 )
            {
                if( _matrix[ x ][ y ].originY == -1 )
                {
                    nodeState.ancestor = AncestorDirection::UPLEFT;
                }
                else if( _matrix[ x ][ y ].originY == 0 )
                {
                    nodeState.ancestor = AncestorDirection::LEFT;
                }
                else
                {
                    nodeState.ancestor = AncestorDirection::DOWNLEFT;
                }
            }
            else if( _matrix[ x ][ y ].originX == 1 )
            {
                if( _matrix[ x ][ y ].originY == -1 )
                {
                    nodeState.ancestor = AncestorDirection::UPRIGHT;
                }
                else if( _matrix[ x ][ y ].originY == 0 )
                {
                    nodeState.ancestor = AncestorDirection::RIGHT;
                }
                else
                {
                    nodeState.ancestor = AncestorDirection::DOWNRIGHT;
                }
            }
            else if( _matrix[ x ][ y ].originX == 0 )
            {
                if( _matrix[ x ][ y ].originY == -1 )
                {
                    nodeState.ancestor = AncestorDirection::UP;
                }
                else if( _matrix[ x ][ y ].originY == 1 )
                {
                    nodeState.ancestor = AncestorDirection::DOWN;
                }
                else
                {
                    nodeState.ancestor = AncestorDirection::SELF;
                }
            }

            newMatrixState->push_back( nodeState );
        }

        _openedNodes.removeOne( *elem );

        if( x > 0 )
        {
            if( y > 0 )
            {
                openNode( x - 1, y - 1, x, y, newMatrixState );
            }

            openNode( x - 1, y, x, y, newMatrixState );

            if( y + 1 < _height )
            {
                openNode( x - 1, y + 1, x, y, newMatrixState );
            }
        }

        if( y > 0 )
        {
            openNode( x, y - 1, x, y, newMatrixState );
        }

        if( y + 1 < _height )
        {
            openNode( x, y + 1, x, y, newMatrixState );
        }

        if( x + 1 < _width )
        {
            if( y > 0 )
            {
                openNode( x + 1, y - 1, x, y, newMatrixState );
            }

            openNode( x + 1, y, x, y, newMatrixState );

            if( y + 1 < _height )
            {
                openNode( x + 1, y + 1, x, y, newMatrixState );
            }
        }

        if( newMatrixState != NULL )
        {
            _matrixsChanges.push_back( *newMatrixState );
        }
    }

    x = _end.first;
    y = _end.second;

    node.first = _matrix[ x ][ y ].originX;
    node.second = _matrix[ x ][ y ].originY;

    while( ! ( _matrix[ x ][ y ].originX == x &&
               _matrix[ x ][ y ].originY == y ) )
    {
        node.first = x;
        node.second = y;

        _path.push_front( node );

        x = _matrix[ node.first ][ node.second ].originX;
        y = _matrix[ node.first ][ node.second ].originY;
    }
}

void AStar::setMatrix( uint width, uint height )
{
    destroyMatrix();

    _width = width;
    _height = height;

    if( width == 0 || height == 0 )
    {
        return;
    }

    _matrix = new SearchNode*[ width ];

    for( uint i = 0; i < width; ++i )
    {
        _matrix[ i ] = new SearchNode[ height ];
    }

    _path.clear();
}

void AStar::setStart( uint x, uint y )
{
    if( x <= _width && y <= _height )
    {
        _start.first = x;
        _start.second = y;

        _path.clear();
    }
}

void AStar::setEnd( uint x, uint y )
{
    if( x <= _width && y <= _height )
    {
        _end.first = x;
        _end.second = y;

        _path.clear();
    }
}

void AStar::setWall( uint x, uint y )
{
    if( _matrix == NULL )
    {
        return;
    }
    else if( x > _width || y > _height )
    {
        return;
    }

    _matrix[ x ][ y ].type = NodeType::WALLNODE;

    _path.clear();
}

void AStar::setWay( uint x, uint y)
{
    if( _matrix == NULL )
    {
        return;
    }
    else if( x > _width || y > _height )
    {
        return;
    }

    _matrix[ x ][ y ].type = NodeType::WAYNODE;

    _path.clear();
}

void AStar::setHeuristics( AStarHeuristics heuristic )
{
    if( heuristic >= AStarHeuristics::none &&
        heuristic <= AStarHeuristics::newH )
    {
        _heuristic = heuristic;
    }
    else
    {
        _heuristic = AStarHeuristics::euclidean;
    }
}

void AStar::setExpandCost( uint expandCost, uint x, uint y )
{
    if( x >= _width || y >= _height )
    {
        return;
    }

    _matrix[ x ][ y ].expandCost = expandCost;
}

void AStar::destroyMatrix()
{
    if( _matrix != NULL )
    {
        for( uint i = 0; i < _width; ++i )
        {
            delete [] _matrix[ i ];
        }

        delete [] _matrix;

        _matrix = NULL;

        _path.clear();
        _matrixsChanges.clear();
    }
}

NodeType AStar::getNodeType( uint x, uint y ) const
{
    if( _matrix == NULL )
    {
        return NodeType::NONE;
    }
    else if( x > _width || y > _height )
    {
        return NodeType::NONE;
    }

    return _matrix[ x ][ y ].type;
}

QPair< uint, uint > AStar::getStart() const
{
    return _start;
}

QPair< uint, uint > AStar::getEnd() const
{
    return _end;
}

QList< QPair< uint, uint > > AStar::getPath( bool saveChanges )
{
    if( _path.size() == 0 )
    {
        search( saveChanges );
    }

    return _path;
}

QList< QList< NodeState > > AStar::getChanges() const
{
    return _matrixsChanges;
}

double AStar::estimateCost( uint x1, uint y1, uint x2, uint y2 )
{
    double x;
    double y;
    double destX;
    double destY;
    double cost;
    double h;
    double euc;
    uint g;

    x = ( float ) x2;
    y = ( float ) y2;
    destX = ( float ) _end.first;
    destY = ( float ) _end.second;

    euc = std::sqrt( std::pow( x - destX, 2.0 ) + std::pow( y - destY, 2.0 ) );

    if( _heuristic == AStarHeuristics::none )
    {
        h = 0;
    }
    else if( _heuristic == AStarHeuristics::euclidean )
    {
        h = euc;
    }
    else if( _heuristic == AStarHeuristics::manhattan )
    {
        h = std::abs( x - destX ) + abs( y - destY );
    }
    else if( _heuristic == AStarHeuristics::diagonal )
    {
        h = std::max(abs(x - destX), abs(y - destY));
    }
    else if( _heuristic == AStarHeuristics::newH )
    {
        h = std::ceil(euc) - euc;
    }

    g = _matrix[ x1 ][ y1 ].g + _matrix[ x2 ][ y2 ].expandCost;

    cost = h + ( double ) g;

    return cost;
}
