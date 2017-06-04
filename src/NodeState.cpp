// Copyright (c) 2016-2017 All Rights Reserved WestBot

#include <WestBot/AStar/NodeState.hpp>

using namespace WestBot::AStar;

NodeState::NodeState()
    : x( 0 )
    , y( 0 )
    , state( NodeType::NONE )
{
}

NodeState::NodeState( uint x, uint y, NodeType state )
    : x( x )
    , y( y )
    , state( state )
    , ancestor( AncestorDirection::SELF )
{
}
