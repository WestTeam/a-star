QT += core
QT -= gui

TARGET = AStar

TEMPLATE = lib

INCLUDEPATH += include

SOURCES += src/AStar.cpp \
           src/NodeState.cpp \
           src/PathNode.cpp \
           src/SearchNode.cpp

HEADERS += include/WestBot/AStar/AStar.hpp \
           include/WestBot/AStar/MapNode.hpp \
           include/WestBot/AStar/NodeState.hpp \
           include/WestBot/AStar/PathNode.hpp \
           include/WestBot/AStar/SearchNode.hpp \
           include/WestBot/AStar/Utils.hpp

DEFINES += WESTBOT_ASTARSHAREDLIB_LIBRARY
