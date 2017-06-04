// Copyright (c) 2016-2017 All Rights Reserved WestBot

#include <WestBot/AStar/SearchNode.hpp>

using namespace WestBot::AStar;

SearchNode::SearchNode()
{
    type = NodeType::WAYNODE;
    originX = 0;
    originY = 0;
    h = std::numeric_limits< double >::max();
    g = std::numeric_limits< uint >::max();
    expandCost = 1;
    cost = std::numeric_limits< double >::max();
    pathCost = std::numeric_limits< double >::max();
}
