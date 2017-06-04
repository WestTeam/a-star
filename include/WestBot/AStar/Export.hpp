// Copyright (c) 2016-2017 All Rights Reserved WestBot

#ifndef WESTBOT_ASTAR_EXPORT_HPP_
#define WESTBOT_ASTAR_EXPORT_HPP_

#include <QtCore/QtGlobal>

#if defined( WESTBOT_ASTARSHAREDLIB_LIBRARY )
#  define WESTBOT_ASTAR_EXPORT Q_DECL_EXPORT
#else
#  define WESTBOT_ASTAR_EXPORT Q_DECL_IMPORT
#endif

#endif // WESTBOT_ASTAR_EXPORT_HPP_
